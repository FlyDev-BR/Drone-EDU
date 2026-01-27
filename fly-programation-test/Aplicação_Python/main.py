# main.py
import webview
import threading
import time
import json
import socket
from pathlib import Path
from pymavlink import mavutil
from drone_api import DroneAPI

try:
    import inputs
except ImportError:
    print("Biblioteca 'inputs' não encontrada. O controle por Joystick estará desativado.")
    print("Para instalar, execute: pip install inputs")
    inputs = None

# --- Constantes ---
# PI_IP_ADDRESS = "192.168.4.1"
PI_IP_ADDRESS = "127.0.0.1"
MAVLINK_PORT = 5760
COMMAND_PORT = 8000
CONNECTION_STRING = f"tcp:{PI_IP_ADDRESS}:{MAVLINK_PORT}"
MANUAL_SPEED_MPS = 0.5
MANUAL_YAW_RATE_DPS = 45
MIN_SAFE_ALTITUDE_METERS = 0.3
TAKEOFF_ALTITUDE_METERS = 1.0
GAMEPAD_CONFIG_FILE = Path('./gamepad_config.json')

try:
    drone = DroneAPI(CONNECTION_STRING)
except Exception as e:
    print(f"Falha ao instanciar a API do drone (MAVLink): {e}")
    drone = None

class ApiFacade:
    def __init__(self, api_real):
        self.api = api_real

    def run_blockly_code(self): return self.api.run_blockly_code()
    def drone_abort_mission(self): return self.api.drone_abort_mission()
    def set_manual_mode(self, active): return self.api.set_manual_mode(active)
    def manual_command(self, direction, key_is_down): return self.api.manual_command(direction, key_is_down)
    def drone_arm(self): return self.api.drone_arm()
    def drone_disarm(self): return self.api.drone_disarm()
    def drone_takeoff(self): return self.api.drone_takeoff()
    def drone_land(self): return self.api.drone_land()
    def drone_set_mode(self, mode_name): return self.api.drone_set_mode(mode_name)
    def drone_calibrate_level(self): return self.api.drone_calibrate_level()
    def drone_reboot_autopilot(self): return self.api.drone_reboot_autopilot()
    def save_mission(self): return self.api.save_mission()
    def load_mission(self): return self.api.load_mission()
    def write_mission_data(self, data): return self.api.write_mission_data(data)
    def save_gamepad_mapping(self, mapping): return self.api.save_gamepad_mapping(mapping)
    def load_gamepad_mapping(self): return self.api.load_gamepad_mapping()
    def restore_default_mapping(self): return self.api.restore_default_mapping()
    def execute_if(self, condition, commands): return self.api.execute_if(condition, commands)
    def send_command_to_pi(self, command_dict): return self.api.send_command_to_pi(command_dict)

class JsApi:
    def __init__(self):
        # Variáveis de estado
        self.manual_control_active = False
        self.velocity_lock = threading.Lock()
        self.vx, self.vy, self.vz, self.yaw_rate = 0, 0, 0, 0
        self.telemetry_lock = threading.Lock()
        self.telemetry_data = { "pitch": 0, "roll": 0, "yaw": 0, "altitude": 0, "ground_speed": 0, "battery_volt": 0, "battery_perc": 0, "gps_fix": 0, "armed": False, "mode": "UNKNOWN" }
        self.mission_lock = threading.Lock()
        self.mission_queue = []
        self.is_mission_running = False
        self.abort_mission_flag = False
        self.window = None
        
        # Lógica do Gamepad
        self.gamepad_mapping = {}
        self.default_gamepad_mapping = {
            'vx': {'code': 'ABS_Y', 'invert': True}, 'vy': {'code': 'ABS_X', 'invert': False},
            'vz': {'code': 'ABS_RZ', 'invert': True}, 'yaw': {'code': 'ABS_Z', 'invert': False}
        }

        # Conexão direta com o servidor de comandos no Pi
        self.command_socket = None
        self.command_lock = threading.Lock()

        # Threads
        self.stop_event = threading.Event()
        self.command_thread = threading.Thread(target=self._command_socket_connect, daemon=True)
        if inputs: self.gamepad_thread = threading.Thread(target=self._gamepad_loop, daemon=True)
        else: self.gamepad_thread = None
        self.manual_control_thread = threading.Thread(target=self._manual_control_loop, daemon=True)
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.mission_runner_thread = threading.Thread(target=self._mission_runner_loop, daemon=True)

    def set_window(self, window):
        self.window = window

    def start_background_tasks(self):
        """Inicia todas as threads de fundo depois que a janela está pronta."""
        print("Janela pronta. Iniciando threads de fundo...")
        self.command_thread.start()
        self.load_gamepad_mapping()
        
        self.manual_control_thread.start()
        self.telemetry_thread.start()
        self.mission_runner_thread.start()
        if self.gamepad_thread:
            self.gamepad_thread.start()

    def _command_socket_connect(self):
        while not self.stop_event.is_set():
            try:
                print(f"Conectando ao servidor de comandos em {PI_IP_ADDRESS}:{COMMAND_PORT}...")
                with self.command_lock:
                    self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.command_socket.connect((PI_IP_ADDRESS, COMMAND_PORT))
                print("Conectado ao servidor de comandos!")
                while not self.stop_event.is_set(): time.sleep(1)
            except Exception as e:
                print(f"Erro no socket de comando: {e}. Tentando novamente em 5s.")
                with self.command_lock:
                    if self.command_socket: self.command_socket.close()
                    self.command_socket = None
                time.sleep(5)
    
    def send_command_to_pi(self, command_dict):
        with self.command_lock:
            if self.command_socket:
                try:
                    message = json.dumps(command_dict)
                    self.command_socket.sendall(message.encode('utf-8'))
                    print(f"Comando enviado para o Pi: {message}")
                except Exception as e:
                    print(f"Falha ao enviar comando para o Pi: {e}")
            else:
                print("Não conectado ao servidor de comandos. Comando descartado.")

    ### FUNÇÃO CORRIGIDA PARA LOOPS ###
    def run_blockly_code(self):
        with self.mission_lock:
            if self.is_mission_running: return
            if not self.window: return

            code_from_blockly = self.window.evaluate_js('getCode()')
            
            print("\n--- CÓDIGO GERADO PELO BLOCKLY ---")
            print(code_from_blockly)
            print("------------------------------------\n")
            
            
            if not code_from_blockly: return

            self.mission_queue = []
            
            # Adiciona setup de voo apenas se houver comandos de drone e o drone estiver conectado
            if drone and any("drone." in line for line in code_from_blockly.splitlines()):
                self.mission_queue.append("drone.set_mode('GUIDED')")
                self.mission_queue.append("time.sleep(1)")

            # Adiciona o CÓDIGO INTEIRO do Blockly como UM ÚNICO item na fila
            self.mission_queue.append(code_from_blockly)

            self.abort_mission_flag = False
            self.is_mission_running = True
            print(f"Missão carregada com {len(self.mission_queue)} blocos de execução.")


    def _mission_runner_loop(self):
        while not self.stop_event.is_set():
            mission_command = None
            with self.mission_lock:
                if self.mission_queue:
                    mission_command = self.mission_queue.pop(0)
            if mission_command:
                if self.abort_mission_flag:
                    print("Missão abortada. Limpando fila e pousando...")
                    with self.mission_lock:
                        self.mission_queue.clear()
                        self.is_mission_running = False
                        self.abort_mission_flag = False
                    if drone: drone.land()
                    continue
                try:
                    print("Executando bloco de comandos da missão...")
                    # print(mission_command) # Descomente para ver o código que está sendo executado
                    exec(mission_command, {"drone": drone, "mission": self, "time": time})
                except Exception as e:
                    print(f"!!! ERRO ao executar bloco de comandos: {e}")
                    with self.mission_lock:
                        self.mission_queue.clear()
                        self.is_mission_running = False
                    if drone: drone.land()
            else:
                if self.is_mission_running:
                    print("Fila de missão concluída.")
                    with self.mission_lock: self.is_mission_running = False
                time.sleep(0.2)
    
    # ... O RESTO DAS SUAS FUNÇÕES (COLEI TODAS PARA GARANTIR) ...
    
    def _telemetry_loop(self):
        while not self.stop_event.is_set():
            if not drone:
                time.sleep(1)
                continue
            while True:
                msg = drone.master.recv_match(type=['ATTITUDE', 'VFR_HUD', 'SYS_STATUS', 'HEARTBEAT', 'GPS_RAW_INT', 'DISTANCE_SENSOR', 'BATTERY_STATUS'], blocking=False)
                if not msg: break
                msg_type = msg.get_type()
                with self.telemetry_lock:
                    if msg_type == 'ATTITUDE':
                        self.telemetry_data['pitch'] = round(msg.pitch * 180.0 / 3.14159, 1)
                        self.telemetry_data['roll'] = round(msg.roll * 180.0 / 3.14159, 1)
                        self.telemetry_data['yaw'] = round(msg.yaw * 180.0 / 3.14159, 1)
                    elif msg_type == 'DISTANCE_SENSOR': self.telemetry_data['altitude'] = round(msg.current_distance / 100.0, 2)
                    elif msg_type == 'VFR_HUD':
                        if self.telemetry_data.get('altitude', 0) == 0: self.telemetry_data['altitude'] = round(msg.alt, 1)
                        if hasattr(msg, 'groundspeed'): self.telemetry_data['ground_speed'] = round(msg.groundspeed, 1)
                    elif msg_type == 'BATTERY_STATUS':
                        if msg.voltages[0] < 65535: self.telemetry_data['battery_volt'] = msg.voltages[0] / 1000.0
                    elif msg_type == 'SYS_STATUS':
                        self.telemetry_data['battery_perc'] = msg.battery_remaining
                        if self.telemetry_data.get('battery_volt', 0) == 0: self.telemetry_data['battery_volt'] = msg.voltage_battery / 1000.0
                    elif msg_type == 'HEARTBEAT':
                        self.telemetry_data['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        self.telemetry_data['mode'] = mavutil.mode_string_v10(msg)
                    elif msg_type == 'GPS_RAW_INT': self.telemetry_data['gps_fix'] = msg.fix_type
            if self.window:
                try: self.window.evaluate_js(f'update_hud({json.dumps(self.telemetry_data)})')
                except Exception: pass
            time.sleep(0.1)

    def _manual_control_loop(self):
        while not self.stop_event.is_set():
            with self.velocity_lock:
                if not self.manual_control_active or not drone:
                    time.sleep(0.1)
                    continue
                vx, vy, vz, yaw_rate = self.vx, self.vy, self.vz, self.yaw_rate
                if vz > 0:
                    with self.telemetry_lock: current_altitude = self.telemetry_data.get('altitude', 0)
                    if current_altitude <= MIN_SAFE_ALTITUDE_METERS: vz = 0
                drone.set_manual_velocity(vx, vy, vz, yaw_rate)
            time.sleep(0.1)

    def _gamepad_loop(self):
        print("Thread do gamepad iniciado.")
        while not self.stop_event.is_set():
            try:
                events = inputs.get_gamepad()
                for event in events:
                    if self.window: self.window.evaluate_js(f'updateGamepadViewer("{event.code}", {event.state})')
                    self._process_gamepad_event(event.code, event.state)
            except inputs.UnpluggedError:
                if self.window: self.window.evaluate_js(f'updateGamepadViewer("DESCONECTADO", 0)')
                time.sleep(2)
            except Exception as e:
                print(f"Erro no loop do gamepad: {e}")
                time.sleep(2)

    def _process_gamepad_event(self, code, state):
        MAX_ABS_VAL, DEADZONE = 32768, 4000
        with self.velocity_lock:
            if not self.manual_control_active: return
            def normalize(value):
                if abs(value) < DEADZONE: return 0.0
                return value / MAX_ABS_VAL
            for action, mapping in self.gamepad_mapping.items():
                if mapping['code'] == code:
                    value = normalize(state)
                    if mapping['invert']: value = -value
                    if action == 'vx': self.vx = value * MANUAL_SPEED_MPS
                    elif action == 'vy': self.vy = value * MANUAL_SPEED_MPS
                    elif action == 'vz': self.vz = value * MANUAL_SPEED_MPS
                    elif action == 'yaw': self.yaw_rate = value * MANUAL_YAW_RATE_DPS
                    break

    def drone_abort_mission(self):
        with self.mission_lock:
            if not self.is_mission_running: return
            self.abort_mission_flag = True

    def set_manual_mode(self, active):
        with self.velocity_lock:
            self.manual_control_active = active
            if not active:
                self.vx, self.vy, self.vz, self.yaw_rate = 0, 0, 0, 0
                if drone: drone.set_manual_velocity(0, 0, 0, 0)
            print(f"Modo de controle manual {'ATIVADO' if active else 'DESATIVADO'}.")

    def manual_command(self, direction, key_is_down):
        value = 1 if key_is_down else 0
        with self.velocity_lock:
            if not self.manual_control_active: return
            if direction == 'forward': self.vx = MANUAL_SPEED_MPS * value
            elif direction == 'backward': self.vx = -MANUAL_SPEED_MPS * value
            elif direction == 'left': self.vy = -MANUAL_SPEED_MPS * value
            elif direction == 'right': self.vy = MANUAL_SPEED_MPS * value
            elif direction == 'up': self.vz = -MANUAL_SPEED_MPS * value
            elif direction == 'down': self.vz = MANUAL_SPEED_MPS * value
            elif direction == 'yaw_left': self.yaw_rate = -MANUAL_YAW_RATE_DPS * value
            elif direction == 'yaw_right': self.yaw_rate = MANUAL_YAW_RATE_DPS * value

    def check_altitude(self, comparison, altitude):
        with self.telemetry_lock: current_altitude = self.telemetry_data.get('altitude', 0)
        if comparison == 'GREATER_THAN': return current_altitude > altitude
        elif comparison == 'LESS_THAN': return current_altitude < altitude
        return False

    def execute_if(self, condition_result, command_string):
        if condition_result:
            print(f"Condição VERDADEIRA. Injetando comandos na fila...")
            commands_to_add = [line.strip() for line in command_string.splitlines() if line.strip()]
            with self.mission_lock: self.mission_queue = commands_to_add + self.mission_queue
        else:
            print("Condição FALSA. Ignorando bloco de comandos.")

    def drone_arm(self):
        if drone: drone.arm()
    def drone_disarm(self):
        if drone: drone.disarm()
    def drone_land(self):
        if drone: drone.land()
    def drone_set_mode(self, mode_name):
        if drone: drone.set_mode(mode_name)
    def drone_calibrate_level(self):
        if drone: drone.calibrate_level()
    def drone_reboot_autopilot(self):
        if drone: drone.reboot_autopilot()

    def drone_takeoff(self):
        threading.Thread(target=self._safe_takeoff_sequence).start()

    def _safe_takeoff_sequence(self):
        try:
            with self.telemetry_lock: current_mode = self.telemetry_data.get('mode', 'UNKNOWN')
            if current_mode != 'LOITER':
                drone.set_mode('LOITER')
                time.sleep(2)
            with self.telemetry_lock: is_armed = self.telemetry_data.get('armed', False)
            if not is_armed:
                self.window.evaluate_js('alert("Decolagem falhou!\\nPor favor, arme o drone primeiro.")')
                return
            drone.takeoff(TAKEOFF_ALTITUDE_METERS)
            target_altitude = TAKEOFF_ALTITUDE_METERS * 0.90
            for _ in range(150):
                with self.telemetry_lock: altitude_atual = self.telemetry_data.get('altitude', 0)
                if altitude_atual >= target_altitude: break
                time.sleep(0.1)
            else:
                print("AVISO: Timeout ao aguardar altitude de decolagem.")
            drone.set_mode('GUIDED')
        except Exception as e: print(f"ERRO GERAL NA SEQUÊNCIA DE DECOLAGEM: {e}")

    def save_mission(self):
        if not self.window: return
        file_path = self.window.create_file_dialog(webview.SAVE_DIALOG, directory='./', save_filename='missao.xml', file_types=('Missões Blockly (*.xml)',))
        if file_path:
            js_safe_path = json.dumps(file_path[0])
            self.window.evaluate_js(f'initiateSave({js_safe_path})')

    def write_mission_data(self, data):
        try:
            with open(data['path'], 'w', encoding='utf-8') as f: f.write(data['content'])
            self.window.evaluate_js('alert("Missão salva com sucesso!")')
        except Exception as e:
            self.window.evaluate_js(f'alert("Erro ao salvar arquivo: {str(e)}")')

    def load_mission(self):
        if not self.window: return
        file_paths = self.window.create_file_dialog(webview.OPEN_DIALOG, allow_multiple=False, file_types=('Missões Blockly (*.xml)',))
        if not file_paths: return
        try:
            with open(file_paths[0], 'r', encoding='utf-8') as f: xml_content = f.read()
            self.window.evaluate_js(f'loadWorkspaceXml({json.dumps(xml_content)})')
        except Exception as e: print(f"Erro ao carregar a missão: {e}")

    def load_gamepad_mapping(self):
        if GAMEPAD_CONFIG_FILE.exists():
            try:
                with open(GAMEPAD_CONFIG_FILE, 'r') as f: self.gamepad_mapping = json.load(f)
            except: self.gamepad_mapping = self.default_gamepad_mapping.copy()
        else: self.gamepad_mapping = self.default_gamepad_mapping.copy()
        if self.window: self.window.evaluate_js(f'populateMappingUi({json.dumps(self.gamepad_mapping)})')

    def save_gamepad_mapping(self, mapping):
        self.gamepad_mapping = mapping
        try:
            with open(GAMEPAD_CONFIG_FILE, 'w') as f: json.dump(self.gamepad_mapping, f, indent=4)
            self.window.evaluate_js('alert("Mapeamento salvo com sucesso!")')
        except Exception as e: self.window.evaluate_js(f'alert("Erro ao salvar mapeamento: {e}")')
    
    def restore_default_mapping(self):
        self.gamepad_mapping = self.default_gamepad_mapping.copy()
        if self.window: self.window.evaluate_js(f'populateMappingUi({json.dumps(self.gamepad_mapping)})')
    
    def shutdown(self):
        print("Desligando...")
        self.stop_event.set()
        with self.command_lock:
            if self.command_socket: self.command_socket.close()
if __name__ == '__main__':
    api_real = JsApi()
    api_facade = ApiFacade(api_real)
    window = webview.create_window(
        'Controle de Drone',
        'index.html',
        js_api=api_facade,
        width=1280, height=800
    )
    api_real.set_window(window)
    
    webview.start(api_real.start_background_tasks, debug=True, gui='edgechromium')
    
    api_real.shutdown()
    print("Aplicação finalizada.")