# main.py

import webview
import threading
import time
import json
from pymavlink import mavutil
from drone_api import DroneAPI

CONNECTION_STRING = "tcp:192.168.4.1:5760"
MANUAL_SPEED_MPS = 0.5
MANUAL_YAW_RATE_DPS = 45

try:
    drone = DroneAPI(CONNECTION_STRING)
except Exception as e:
    print(f"Falha ao instanciar a API do drone: {e}")
    drone = None

class JsApi:
    def __init__(self):
        self.manual_control_active = False
        self.velocity_lock = threading.Lock()
        self.vx, self.vy, self.vz, self.yaw_rate = 0, 0, 0, 0
        self.telemetry_lock = threading.Lock()
        self.telemetry_data = { "pitch": 0, "roll": 0, "yaw": 0, "altitude": 0, "ground_speed": 0, "battery_volt": 0, "battery_perc": 0, "gps_fix": 0, "armed": False, "mode": "UNKNOWN" }
        self.stop_event = threading.Event()
        self.manual_control_thread = threading.Thread(target=self._manual_control_loop)
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop)
        self.mission_lock = threading.Lock()
        self.mission_queue = []
        self.is_mission_running = False
        self.abort_mission_flag = False
        self.mission_runner_thread = threading.Thread(target=self._mission_runner_loop)
        self.window = None
        self.manual_control_thread.start()
        self.telemetry_thread.start()
        self.mission_runner_thread.start()

    def set_window(self, window):
        """Recebe e armazena a instância da janela principal."""
        self.window = window

    def _telemetry_loop(self):
        while not self.stop_event.is_set():
            if not drone:
                time.sleep(1)
                continue
            msg = drone.master.recv_match(type=['ATTITUDE', 'VFR_HUD', 'SYS_STATUS', 'HEARTBEAT', 'GPS_RAW_INT'], blocking=False, timeout=0.05)
            if msg:
                msg_type = msg.get_type()
                with self.telemetry_lock:
                    if msg_type == 'ATTITUDE':
                        self.telemetry_data['pitch'] = round(msg.pitch * 180.0 / 3.14159, 1)
                        self.telemetry_data['roll'] = round(msg.roll * 180.0 / 3.14159, 1)
                    elif msg_type == 'VFR_HUD':
                        self.telemetry_data['altitude'] = round(msg.alt, 1)
                        if hasattr(msg, 'groundspeed'):
                            self.telemetry_data['ground_speed'] = round(msg.groundspeed, 1)
                    elif msg_type == 'SYS_STATUS':
                        self.telemetry_data['battery_volt'] = msg.voltage_battery / 1000.0
                        self.telemetry_data['battery_perc'] = msg.battery_remaining
                    elif msg_type == 'HEARTBEAT':
                        self.telemetry_data['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        self.telemetry_data['mode'] = mavutil.mode_string_v10(msg)
                    elif msg_type == 'GPS_RAW_INT':
                        self.telemetry_data['gps_fix'] = msg.fix_type
            if self.window:
                json_data = json.dumps(self.telemetry_data)
                self.window.evaluate_js(f'update_hud({json_data})')
            time.sleep(0.05)

    def _mission_runner_loop(self):
        while not self.stop_event.is_set():
            mission_command = None
            with self.mission_lock:
                if self.mission_queue:
                    mission_command = self.mission_queue.pop(0)
            if mission_command:
                if self.abort_mission_flag:
                    print("Missão abortada pelo usuário. Limpando fila e pousando...")
                    with self.mission_lock:
                        self.mission_queue.clear()
                        self.is_mission_running = False
                        self.abort_mission_flag = False
                    if drone: drone.land()
                    continue
                try:
                    print(f"Executando comando da missão: {mission_command}")
                    exec(mission_command, {"drone": drone})
                except Exception as e:
                    print(f"!!! ERRO ao executar comando da missão '{mission_command}': {e}")
                    with self.mission_lock:
                        self.mission_queue.clear()
                        self.is_mission_running = False
                    if drone: drone.land()
            else:
                if self.is_mission_running:
                    print("Fila de missão concluída.")
                    with self.mission_lock:
                        self.is_mission_running = False
                time.sleep(0.2)

    def _manual_control_loop(self):
        while not self.stop_event.is_set():
            with self.velocity_lock:
                if self.manual_control_active and drone:
                    if any(v != 0 for v in [self.vx, self.vy, self.vz, self.yaw_rate]):
                         drone.set_manual_velocity(self.vx, self.vy, self.vz, self.yaw_rate)
            time.sleep(0.1)
    
    def run_blockly_code(self):
        print("\n--- Recebida Nova Missão ---")
        with self.mission_lock:
            if self.is_mission_running:
                print("ERRO: Uma missão já está em andamento.")
                return
            if not drone:
                print("ERRO: Drone não conectado.")
                return
            code_from_blockly = window.evaluate_js('getCode()')
            if not code_from_blockly:
                print("Código do Blockly está vazio.")
                return
            self.abort_mission_flag = False
            self.mission_queue = [line for line in code_from_blockly.splitlines() if line.strip()]
            self.is_mission_running = True
            print(f"Missão carregada com {len(self.mission_queue)} comandos. Iniciando execução...")

    def drone_abort_mission(self):
        print("!!! COMANDO DE ABORTAR MISSÃO RECEBIDO !!!")
        with self.mission_lock:
            if not self.is_mission_running:
                print("Nenhuma missão em andamento para abortar.")
                return
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

    def drone_arm_disarm(self):
        if drone:
            with self.telemetry_lock: is_armed = self.telemetry_data.get('armed', False)
            if is_armed: drone.disarm()
            else: drone.arm()

    def drone_takeoff(self):
        if drone: drone.takeoff(1)

    def drone_land(self):
        if drone: drone.land()

    def drone_set_mode(self, mode_name):
        if drone: drone.set_mode(mode_name)
    
    def drone_calibrate_level(self):
        if drone: drone.calibrate_level()

    def drone_reboot_autopilot(self):
        if drone: drone.reboot_autopilot()
    
    def shutdown(self):
        print("Desligando...")
        self.stop_event.set()
        if self.manual_control_thread.is_alive(): self.manual_control_thread.join()
        if self.telemetry_thread.is_alive(): self.telemetry_thread.join()
        if self.mission_runner_thread.is_alive(): self.mission_runner_thread.join()
        print("Threads finalizados.")

if __name__ == '__main__':
    api = JsApi()
    window = webview.create_window(
        'Controle de Drone',
        'index.html',
        js_api=api
    )
    # LINHA CRÍTICA ADICIONADA AQUI:
    api.set_window(window)

    window.events.closed += api.shutdown
    webview.start(debug=True)