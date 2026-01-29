# main.py
import webview
import threading
import time
import json
import socket
from pathlib import Path
from queue import Queue, Empty
from pymavlink import mavutil
from drone_api import DroneAPI

try:
    import inputs
except ImportError:
    print("Biblioteca 'inputs' não encontrada. O controle por Joystick estará desativado.")
    inputs = None

# --- Constantes ---
PI_IP_ADDRESS = "127.0.0.1"
MAVLINK_PORT = 5760
COMMAND_PORT = 8000
CONNECTION_STRING = f"tcp:{PI_IP_ADDRESS}:{MAVLINK_PORT}"
MANUAL_SPEED_MPS = 0.5
MANUAL_YAW_RATE_DPS = 45
MIN_SAFE_ALTITUDE_METERS = 0.3
TAKEOFF_ALTITUDE_METERS = 1.0
MAX_WHILE_ATTEMPTS = 50
GAMEPAD_CONFIG_FILE = Path('./gamepad_config.json')

try:
    drone = DroneAPI(CONNECTION_STRING)
except Exception as e:
    print(f"Falha ao instanciar a API do drone (MAVLink): {e}")
    drone = None


class ApiFacade:
    def __init__(self, api_real):
        self.api = api_real

    def run_blockly_code(self, code): return self.api.run_blockly_code(code)
    def abort_mission(self): return self.api.abort_mission()
    def set_manual_mode(self, active): return self.api.set_manual_mode(active)
    def manual_command(self, direction, key_is_down): return self.api.manual_command(direction, key_is_down)
    def drone_arm(self): return self.api.drone_arm()
    def drone_disarm(self): return self.api.drone_disarm()
    def drone_takeoff(self, altitude): return self.api.drone_takeoff(altitude)
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
        self.window = None
        self.ui_ready = False
        self.last_ui_update = 0
        self.stop_event = threading.Event()
        self.abort_event = threading.Event()
        self.ui_ready = False
        self.last_ui_update = 0
        self.velocity_lock = threading.Lock()
        self.telemetry_lock = threading.Lock()
        self.mission_lock = threading.Lock()

        self.manual_control_active = False
        self.vx = self.vy = self.vz = self.yaw_rate = 0

        self.telemetry_data = {
            "pitch": 0, "roll": 0, "yaw": 0, "altitude": 0,
            "ground_speed": 0, "battery_volt": 0,
            "battery_perc": 0, "gps_fix": 0,
            "armed": False, "mode": "UNKNOWN"
        }

        self.mission_queue = []
        self.is_mission_running = False

        self.js_queue = Queue()

        self.gamepad_mapping = {}
        self.default_gamepad_mapping = {
            'vx': {'code': 'ABS_Y', 'invert': True},
            'vy': {'code': 'ABS_X', 'invert': False},
            'vz': {'code': 'ABS_RZ', 'invert': True},
            'yaw': {'code': 'ABS_Z', 'invert': False}
        }

        self.command_socket = None
        self.command_lock = threading.Lock()

        self.command_thread = threading.Thread(target=self._command_socket_connect, daemon=True)
        self.js_thread = threading.Thread(target=self._js_dispatch_loop, daemon=True)
        self.manual_control_thread = threading.Thread(target=self._manual_control_loop, daemon=True)
        self.mission_runner_thread = threading.Thread(target=self._mission_runner_loop, daemon=True)
        self.gamepad_thread = threading.Thread(target=self._gamepad_loop, daemon=True) if inputs else None

    # ===================== JS SAFE =====================
    def call_js(self, script):
        if not self.ui_ready or not self.window:
            return

        now = time.time()
        if now - self.last_ui_update < 0.2:
            return

        self.last_ui_update = now
        self.js_queue.put(script)

    def _js_dispatch_loop(self):
        while not self.stop_event.is_set():
            try:
                script = self.js_queue.get(timeout=0.1)
                if self.window:
                    self.window.evaluate_js(script)  # ✅ CORREÇÃO AQUI
            except Empty:
                pass
            except Exception as e:
                print(f"Erro JS: {e}")

    # ===================== SOCKET =====================
    def _command_socket_connect(self):
        while not self.stop_event.is_set():
            try:
                with self.command_lock:
                    self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.command_socket.connect((PI_IP_ADDRESS, COMMAND_PORT))
                print("Conectado ao servidor de comandos!")
                while not self.stop_event.is_set():
                    time.sleep(1)
            except Exception as e:
                print(f"Erro socket: {e}")
                time.sleep(5)

    # ===================== START =====================
    def set_window(self, window):
        self.window = window

    def start_background_tasks(self):
        print("Janela pronta. Iniciando threads...")
        self.ui_ready = True

        self.js_thread.start()
        self.command_thread.start()
        self.load_gamepad_mapping()
        self.manual_control_thread.start()
        self.mission_runner_thread.start()

        if self.gamepad_thread:
            self.gamepad_thread.start()

    # ===================== BLOCKLY =====================
    def run_blockly_code(self, code):
        with self.mission_lock:
            if self.is_mission_running or not code:
                return

            self.mission_queue.clear()

            if drone and "drone." in code:
                self.mission_queue.extend([
                    "drone.set_mode('GUIDED')",
                    "mission.wait(1)"
                ])

            self.mission_queue.append(code)
            self.is_mission_running = True
            print("Missão Blockly iniciada")

    def _mission_runner_loop(self):
        TELEMETRY_INTERVAL = 0.2  
        last_telemetry_update = 0
        while not self.stop_event.is_set():
            try:
                self.check_abort()

                now = time.time()
                if now - last_telemetry_update > TELEMETRY_INTERVAL:
                    self.update_telemetry()
                    last_telemetry_update = now

                cmd = None
                with self.mission_lock:
                    if self.mission_queue:
                        cmd = self.mission_queue.pop(0)

                if cmd:
                    stripped = cmd.rstrip()
                    if stripped.endswith(":"):
                        cmd += "\n    pass"

                    if stripped.lstrip().startswith("while "):
                        cmd = cmd.replace(
                            "while ",
                            "while ",
                            1
                        )
                        cmd += "\n    mission.while_guard(__while_counter__)"

                    exec_context = {
                        "drone": drone,
                        "mission": self,
                        "abort_event": self.abort_event,
                        "__while_counter__": {"count": 0}
                    }

                    exec(cmd, exec_context)

                else:
                    if self.is_mission_running:
                        self.is_mission_running = False
                    time.sleep(0.1)

            except RuntimeError as e:
                if str(e) == "MISSION_ABORTED":
                    print("🛑 Missão abortada com segurança")
                    if drone:
                        drone.land()
                        time.sleep(1)
                        drone.disarm()

                    self.is_mission_running = False
                    self.mission_queue.clear()
                    self.abort_event.clear()

    def execute_if(self, condition, commands):
        self.check_abort()

        if not condition:
            return
        try:
            exec(commands, {
                "drone": drone,
                "mission": self,
                "abort_event": self.abort_event,
                "check_abort": self.check_abort
            })
        except RuntimeError as e:
            if str(e) == "MISSION_ABORTED":
                raise

    def repeat_ext(self, times, body_fn):
        """
        Executa um loop seguro por um número específico de vezes.
        Checa abort a cada iteração.
        Limita a 50 ou 10 iterações extras, o que for menor.
        """
        max_limit = min(times + 10, 50)  # não mais que 50, nem mais que 10 extras
        iteration = 0

        while iteration < max_limit:
            self.check_abort() 

            if iteration >= times:
                print(f"⚠️ Repetição extra segura: {iteration+1}/{max_limit}")

            try:
                body_fn()
            except RuntimeError as e:
                if str(e) == "MISSION_ABORTED":
                    print("🛑 Loop interrompido pelo abort")
                    break

            iteration += 1

    def while_until_safe(self, condition_fn, body_fn, max_iters=50):
        """
        Loop while/até seguro:
        - Checa abort a cada iteração
        - Limita a no máximo `max_iters` iterações
        - `condition_fn()` deve retornar True para continuar
        - `body_fn()` é o código dentro do loop
        """
        iteration = 0
        print("[Mission] while_until_safe iniciado")
        while condition_fn():
            self.check_abort()  

            if iteration >= max_iters:
                print(f"🛑 Limite máximo de iterações atingido ({max_iters})")
                self.abort_event.set()
                raise RuntimeError("MISSION_ABORTED")

            try:
                body_fn()
            except RuntimeError as e:
                if str(e) == "MISSION_ABORTED":
                    print("🛑 Loop interrompido pelo abort")
                    break

            iteration += 1

    def execute_for(self, var_name, start, end, step, commands):
        """
        Executa o bloco FOR do Blockly de forma segura.
        - var_name: nome da variável do loop
        - start, end, step: valores do range
        - commands: código Python gerado pelo Blockly
        """
        def body(i):
            self.check_abort()
            try:
                exec(commands, {
                    "drone": drone,
                    "mission": self,
                    "abort_event": self.abort_event,
                    "check_abort": self.check_abort,
                    var_name: i
                })
            except RuntimeError as e:
                if str(e) == "MISSION_ABORTED":
                    raise

        # Ajusta range para não quebrar se step for negativo
        if step == 0:
            step = 1
        if step > 0:
            rng = range(start, end + 1, step)
        else:
            rng = range(start, end - 1, step)

        for i in rng:
            body(i)


    def abort_mission(self):
        print("🚨 ABORT solicitado pelo usuário")
        self.abort_event.set()
        self.is_mission_running = False
        self.mission_queue.clear()
        
        try:
            if drone:
                drone.land()
        except:
            pass


    def check_abort(self):
        if self.abort_event.is_set():
            print("🚨 Missão abortada imediatamente")
            raise RuntimeError("MISSION_ABORTED")

    # ===================== MANUAL =====================
    def _manual_control_loop(self):
        while not self.stop_event.is_set():
            with self.velocity_lock:
                if self.manual_control_active and drone:
                    drone.set_manual_velocity(self.vx, self.vy, self.vz, self.yaw_rate)
            time.sleep(0.1)

    # ===================== GAMEPAD =====================
    def _gamepad_loop(self):
        DEADZONE = 0.1  # ±10% ignorado
        MAX_AXIS = 32767  # valor máximo do eixo

        while not self.stop_event.is_set():
            try:
                for e in inputs.get_gamepad():
                    self.call_js(f'updateGamepadViewer("{e.code}", {e.state})')

                    # Processamento de joystick
                    for axis_name, cfg in self.gamepad_mapping.items():
                        if e.code == cfg['code']:
                            val = e.state / MAX_AXIS
                            if cfg.get('invert', False):
                                val = -val
                            # deadzone
                            if abs(val) < DEADZONE:
                                val = 0
                            setattr(self, axis_name, val)
            except:
                time.sleep(0.1)

    # ===================== HELPERS =====================
    def drone_arm(self):
        if drone: drone.arm()

    def drone_disarm(self):
        if drone: drone.disarm()

    def drone_land(self):
        if drone: drone.land()

    def drone_calibrate_level(self):
        if drone:
            drone.calibrate_level()

    def drone_reboot_autopilot(self):
        if drone:
            drone.reboot_autopilot()

    def wait(self, seconds):
        steps = int(seconds * 10)
        for _ in range(steps):
            self.check_abort()
            time.sleep(0.1)

    # metodo de decolagem para testes
    def drone_takeoff(self, altitude):
        if drone: drone.takeoff(altitude)

    # metodo de decolagem seguro
    # def drone_takeoff(self, target_altitude):
    #     """
    #     Takeoff seguro:
    #     - Garante que o drone está armado
    #     - Timeout de segurança
    #     - Checa altitude mínima antes de iniciar
    #     """
    #     if not drone:
    #         print("🚨 Drone não disponível")
    #         return

    #     current_alt = drone.get_altitude()
    #     if current_alt > MIN_SAFE_ALTITUDE_METERS:
    #         print(f"Drone já acima da altitude mínima ({current_alt:.2f}m)")
        
    #     if not drone.is_armed():
    #         print("Armando drone...")
    #         drone.arm()
    #         arm_timeout = 5
    #         start_time = time.time()
    #         while not drone.is_armed():
    #             self.check_abort()
    #             if time.time() - start_time > arm_timeout:
    #                 raise RuntimeError("TIMEOUT_ARMING")
    #             time.sleep(0.1)
    #         print("✅ Drone armado")

    #     drone.set_mode("GUIDED")

    #     print(f"Decolando até {target_altitude} m...")
    #     drone.takeoff(target_altitude)

    #     takeoff_timeout = 10
    #     start_time = time.time()
    #     while True:
    #         self.check_abort()
    #         alt = drone.get_altitude()
    #         print(f"[Takeoff] Altitude atual: {alt:.2f} m")
    #         if alt >= target_altitude * 0.95:
    #             print("✅ Altitude de decolagem atingida")
    #             break
    #         if time.time() - start_time > takeoff_timeout:
    #             raise RuntimeError("TIMEOUT_TAKEOFF")
    #         time.sleep(0.2)

    def save_mission(self):
        if not self.window:
            return
        file_path = self.window.create_file_dialog(
            webview.SAVE_DIALOG,
            directory='./',
            save_filename='missao.xml',
            file_types=('Missões Blockly (*.xml)',)
        )
        if file_path:
            js_safe_path = json.dumps(file_path[0])
            # Agora usamos call_js em vez de evaluate_js direto
            self.call_js(f'initiateSave({js_safe_path})')

    def write_mission_data(self, data):
        try:
            with open(data['path'], 'w', encoding='utf-8') as f:
                f.write(data['content'])
            self.call_js('alert("Missão salva com sucesso!")')
        except Exception as e:
            self.call_js(f'alert("Erro ao salvar arquivo: {str(e)}")')

    def load_mission(self):
        if not self.window:
            return
        file_paths = self.window.create_file_dialog(
            webview.OPEN_DIALOG,
            allow_multiple=False,
            file_types=('Missões Blockly (*.xml)',)
        )
        if not file_paths:
            return
        try:
            with open(file_paths[0], 'r', encoding='utf-8') as f:
                xml_content = f.read()
            # Usando call_js
            self.call_js(f'loadWorkspaceXml({json.dumps(xml_content)})')
        except Exception as e:
            print(f"Erro ao carregar a missão: {e}")


    def load_gamepad_mapping(self):
        if GAMEPAD_CONFIG_FILE.exists():
            with open(GAMEPAD_CONFIG_FILE) as f:
                self.gamepad_mapping = json.load(f)
        else:
            self.gamepad_mapping = self.default_gamepad_mapping.copy()

        self.call_js(f'populateMappingUi({json.dumps(self.gamepad_mapping)})')

    def shutdown(self):
        self.stop_event.set()
        with self.command_lock:
            if self.command_socket:
                self.command_socket.close()

    def check_altitude(self, operator, value):
        if not drone:
            return False

        altitude = drone.get_altitude()

        if operator == ">":
            return altitude > value
        elif operator == "<":
            return altitude < value
        elif operator == ">=":
            return altitude >= value
        elif operator == "<=":
            return altitude <= value
        elif operator == "==":
            return altitude == value

        return False

    def safe_while_altitude(self, operator, target, body_fn, max_iters=20):
        """
        Executa um loop controlado por altitude com proteção anti-loop infinito
        """
        iters = 0
        while True:
            self.check_abort()

            altitude = drone.get_altitude()
            print(f"[ALTITUDE CHECK] {altitude:.2f} m {operator} {target}")

            condition = {
                "<": altitude < target,
                "<=": altitude <= target,
                ">": altitude > target,
                ">=": altitude >= target,
                "==": altitude == target,
            }.get(operator, False)

            if not condition:
                break

            body_fn()

            iters += 1
            if iters >= max_iters:
                raise RuntimeError("ALTITUDE_LOOP_SAFETY_BREAK")

    def while_guard(self, counter):
        counter["count"] += 1
        if counter["count"] > 50:
            print("🛑 Limite máximo de tentativas atingido (50)")
            self.abort_event.set()
            raise RuntimeError("MISSION_ABORTED")

    def update_telemetry(self):
        if not drone:
            return

        with self.telemetry_lock:
            self.telemetry_data.update({
                "pitch": drone.get_pitch(),
                "roll": drone.get_roll(),
                "yaw": drone.get_yaw(),
                "altitude": drone.get_altitude(),
                "ground_speed": drone.get_ground_speed(),
                "battery_volt": drone.get_battery_voltage(),
                "battery_perc": drone.get_battery_percentage(),
                "gps_fix": drone.get_gps_fix(),
                "armed": drone.is_armed(),
                "mode": drone.get_mode()
            })
        self.call_js(f'updateTelemetry({json.dumps(self.telemetry_data)})')

    
if __name__ == '__main__':
    api_real = JsApi()
    api_facade = ApiFacade(api_real)

    window = webview.create_window(
        "Controle de Drone",
        "index.html",
        js_api=api_facade,
        width=1280,
        height=800
    )

    api_real.set_window(window)
    webview.start(api_real.start_background_tasks, debug=False)
    api_real.shutdown()
