import os
import sys
import threading
import time
import json
import socket
import uvicorn
from pathlib import Path
from queue import Queue, Empty

# Bibliotecas de Interface e Web
import webview
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware

# Bibliotecas do Drone
from drone_api import DroneAPI

# Tenta importar inputs para Gamepad
try:
    import inputs
except ImportError:
    print("⚠️ Biblioteca 'inputs' não encontrada. O controle por Joystick estará desativado.")
    inputs = None

# --- CONSTANTES GLOBAIS ---
SERVER_PORT = 3000  # Porta interna do servidor
PI_IP_ADDRESS = "127.0.0.1"
MAVLINK_PORT = 5760
COMMAND_PORT = 8000
CONNECTION_STRING = f"tcp:{PI_IP_ADDRESS}:{MAVLINK_PORT}"

MANUAL_SPEED_MPS = 0.5
MANUAL_YAW_RATE_DPS = 45
GAMEPAD_CONFIG_FILE = Path('./gamepad_config.json')

# --- CONFIGURAÇÃO PARA EXECUTÁVEL (PyInstaller) ---
def resource_path(relative_path):
    """ Retorna o caminho absoluto, compatível com PyInstaller e Dev """
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.abspath("."), relative_path)

# --- CONFIGURAÇÃO DO FASTAPI ---
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Servir arquivos estáticos do React (pasta dist)
# Certifique-se de que a pasta 'dist' esteja ao lado do main.py
static_dist_path = resource_path("dist")

# Verifica se a pasta dist existe para evitar crash imediato
if os.path.exists(static_dist_path):
    app.mount("/assets", StaticFiles(directory=os.path.join(static_dist_path, "assets")), name="assets")
else:
    print(f"⚠️ AVISO: Pasta 'dist' não encontrada em: {static_dist_path}")
    print("   Execute 'npm run build' no React e copie a pasta dist para cá.")

@app.get("/{full_path:path}")
async def serve_react_app(full_path: str):
    """Route catch-all para suportar React Router (SPA)"""
    # Se pedir um arquivo que existe na raiz do dist (ex: vite.svg), serve ele
    potential_file = os.path.join(static_dist_path, full_path)
    if os.path.exists(potential_file) and os.path.isfile(potential_file):
        return FileResponse(potential_file)
    
    # Caso contrário, retorna o index.html para o React tratar a rota
    index_path = os.path.join(static_dist_path, "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    return {"error": "Frontend files not found"}

# --- CLASSES DE CONTROLE ---

class ApiFacade:
    """
    Ponte entre o JavaScript e o Python. 
    O PyWebview expõe os métodos desta classe para o window.pywebview.api
    """
    def __init__(self, controller):
        self.controller = controller

    def run_blockly_code(self, code): return self.controller.run_blockly_code(code)
    def abort_mission(self): return self.controller.abort_mission()
    def set_manual_mode(self, active): return self.controller.set_manual_mode(active)
    def manual_command(self, direction, key_is_down): return self.controller.manual_command(direction, key_is_down)
    def drone_arm(self): return self.controller.drone_arm()
    def drone_disarm(self): return self.controller.drone_disarm()
    def drone_takeoff(self, altitude): return self.controller.drone_takeoff(altitude)
    def drone_land(self): return self.controller.drone_land()
    def drone_set_mode(self, mode_name): return self.controller.drone_set_mode(mode_name)
    def drone_reboot_autopilot(self): return self.controller.drone_reboot_autopilot()
    def save_mission(self): return self.controller.save_mission()
    def load_mission(self): return self.controller.load_mission()   
    def write_mission_data(self, data): return self.controller.write_mission_data(data)
    def save_gamepad_mapping(self, mapping): return self.controller.save_gamepad_mapping(mapping)
    def load_gamepad_mapping(self): return self.controller.load_gamepad_mapping()
    def restore_default_mapping(self): return self.controller.restore_default_mapping()
    # Adicione outros métodos conforme necessário

class DroneBackendController:
    """
    Controlador principal da lógica de negócios.
    Gerencia conexões, threads e estado do drone.
    """
    def __init__(self):
        self.window = None
        self.drone = None
        self.ui_ready = False
        self.stop_event = threading.Event()
        self.abort_event = threading.Event()
        
        # Locks para thread safety
        self.velocity_lock = threading.Lock()
        self.telemetry_lock = threading.Lock()
        self.mission_lock = threading.Lock()
        self.command_lock = threading.Lock()

        # Estado do Drone/Missão
        self.manual_control_active = False
        self.is_mission_running = False
        self.mission_queue = []
        self.vx = self.vy = self.vz = self.yaw_rate = 0
        self.js_queue = Queue()

        self.telemetry_data = {
            "pitch": 0, "roll": 0, "yaw": 0, "altitude": 0,
            "ground_speed": 0, "battery_volt": 0, "battery_perc": 0,
            "gps_fix": 0, "armed": False, "mode": "DISCONNECTED"
        }

        # Gamepad
        self.gamepad_mapping = {}
        self.default_gamepad_mapping = {
            'vx': {'code': 'ABS_Y', 'invert': True},
            'vy': {'code': 'ABS_X', 'invert': False},
            'vz': {'code': 'ABS_RZ', 'invert': True},
            'yaw': {'code': 'ABS_Z', 'invert': False}
        }

        # Socket Pi
        self.command_socket = None

        # Inicializa conexão com o Drone
        self._init_drone_connection()

    def _init_drone_connection(self):
        try:
            self.drone = DroneAPI(CONNECTION_STRING)
        except Exception as e:
            print(f"❌ Falha ao conectar MAVLink: {e}")
            self.drone = None

    def set_window(self, window):
        self.window = window

    def log_to_frontend(self, message):
        """Envia mensagem para o componente LogDisplay do React"""
        print(f"[BACKEND LOG] {message}") # Print no terminal do Python
        # Escapa aspas para evitar erro de JS
        safe_message = message.replace('"', "'").replace('\n', ' ')
        self.call_js(f'updateLogs("{safe_message}")')
    
    # ===================== JS HELPER =====================
    def call_js(self, script):
        """Envia comandos JS para a UI de forma thread-safe"""
        if not self.ui_ready or not self.window:
            return
        self.js_queue.put(script)

    def _process_js_queue(self):
        """Loop que consome a fila de comandos JS e executa na window"""
        if not self.window: return
        try:
            while True:
                script = self.js_queue.get_nowait()
                self.window.evaluate_js(script)
        except Empty:
            pass
        except Exception as e:
            print(f"Erro JS Thread: {e}")

    # ===================== THREADS =====================
    def start_background_tasks(self):
        print("✅ Backend pronto. Iniciando loops...")
        self.ui_ready = True
        
        # Inicia threads
        threads = [
            threading.Thread(target=self._command_socket_loop, daemon=True),
            threading.Thread(target=self._manual_control_loop, daemon=True),
            threading.Thread(target=self._mission_runner_loop, daemon=True)
        ]

        if inputs:
            self.load_gamepad_mapping()
            threads.append(threading.Thread(target=self._gamepad_loop, daemon=True))

        for t in threads:
            t.start()

        # Loop principal da thread de interface (JS Consumer)
        while not self.stop_event.is_set():
            self._process_js_queue()
            time.sleep(0.05)

    def shutdown(self):
        print("🛑 Encerrando backend...")
        self.stop_event.set()
        if self.command_socket:
            self.command_socket.close()

    # ===================== LOGICA DO DRONE (Exemplos) =====================
    def drone_arm(self):
        if self.drone: self.drone.arm()
    
    def drone_disarm(self):
        if self.drone: self.drone.disarm()

    def drone_takeoff(self, altitude):
        if self.drone: self.drone.takeoff(float(altitude))

    def drone_land(self):
        if self.drone: self.drone.land()

    def drone_set_mode(self, mode):
        if self.drone: self.drone.set_mode(mode)

    def drone_reboot_autopilot(self):
        if self.drone: self.drone.reboot_autopilot()

    # ===================== TELEMETRIA =====================
    def update_telemetry(self):
        if not self.drone:
            return

        with self.telemetry_lock:
            # Atualiza dicionário
            self.telemetry_data.update({
                "pitch": self.drone.get_pitch(),
                "roll": self.drone.get_roll(),
                "yaw": self.drone.get_yaw(),
                "altitude": self.drone.get_altitude(),
                "ground_speed": self.drone.get_ground_speed(),
                "battery_volt": self.drone.get_battery_voltage(),
                "battery_perc": self.drone.get_battery_percentage(),
                "gps_fix": self.drone.get_gps_fix(),
                "armed": self.drone.is_armed(),
                "mode": self.drone.get_mode()
            })
        
        # Envia para o React
        self.call_js(f'updateTelemetry({json.dumps(self.telemetry_data)})')

    # ===================== MISSÃO E LOOPS =====================
    def _mission_runner_loop(self):
        TELEMETRY_INTERVAL = 0.2
        last_telemetry = 0

        # Contexto para execução de código dinâmico (Blockly)
        exec_context = {
            "drone": self.drone,
            "mission": self,
            "abort_event": self.abort_event,
            "check_abort": self.check_abort,
            "time": time
        }

        while not self.stop_event.is_set():
            # 1. Telemetria constante
            now = time.time()
            if now - last_telemetry >= TELEMETRY_INTERVAL:
                self.update_telemetry()
                last_telemetry = now

            # 2. Execução de Missão
            code = None
            with self.mission_lock:
                if self.mission_queue:
                    code = self.mission_queue.pop(0)
            
            if code:
                try:
                    if not self.drone:
                        raise Exception("Drone desconectado")

                    # Atualiza o contexto com o objeto drone atual (caso tenha reconectado)
                    self.log_to_frontend("Executando bloco de código...")
                    exec_context["drone"] = self.drone
                    exec(code, exec_context)
                    
                    print("✅ Bloco de missão executado.")
                    self.is_mission_running = False
                except Exception as e:
                    print(f"❌ Erro na missão: {e}")
                    clean_err = str(e).replace('"', "'").replace('\n', ' ')
                    self.call_js(f'alert("Erro Missão: {clean_err}")')
                    self.is_mission_running = False
                    self.abort_event.clear()
            else:
                time.sleep(0.1)

    def run_blockly_code(self, code):
        """Recebe código Python puro gerado pelo Blockly no Frontend"""
        if self.is_mission_running:
            self.call_js('alert("Missão já em execução!")')
            return

        self.log_to_frontend("Recebendo nova missão...")

        header = "mission.check_abort()\n"
        full_code = header + code
        
        with self.mission_lock:
            self.mission_queue.clear()
            self.abort_event.clear()
            self.mission_queue.append(full_code)
            self.is_mission_running = True
            print("🚀 Missão recebida e enfileirada.")

    def abort_mission(self):
        print("🚨 ABORTAR!")
        self.abort_event.set()
        self.is_mission_running = False
        self.mission_queue.clear()
        if self.drone:
            try: self.drone.land()
            except: pass

    def check_abort(self):
        if self.abort_event.is_set():
            raise RuntimeError("MISSION_ABORTED")

    # ===================== MANUAL CONTROL =====================
    def set_manual_mode(self, active):
        self.manual_control_active = active
        if self.drone:
            if active:
                self.drone.set_mode("GUIDED")
            else:
                self.drone.set_manual_velocity(0,0,0,0)

    def manual_command(self, direction, active):
        speed = MANUAL_SPEED_MPS
        yaw_speed = MANUAL_YAW_RATE_DPS
        
        # Reset velocities based on direction release
        if not active:
            if direction in ['forward', 'backward']: self.vx = 0
            if direction in ['left', 'right']: self.vy = 0
            if direction in ['up', 'down']: self.vz = 0
            if direction in ['yaw_left', 'yaw_right']: self.yaw_rate = 0
            return

        # Set velocities
        if direction == 'forward': self.vx = speed
        if direction == 'backward': self.vx = -speed
        if direction == 'left': self.vy = -speed
        if direction == 'right': self.vy = speed
        if direction == 'up': self.vz = -speed   # NED: negativo é pra cima
        if direction == 'down': self.vz = speed
        if direction == 'yaw_left': self.yaw_rate = -yaw_speed
        if direction == 'yaw_right': self.yaw_rate = yaw_speed

    def _manual_control_loop(self):
        while not self.stop_event.is_set():
            with self.velocity_lock:
                if self.manual_control_active and self.drone:
                    self.drone.set_manual_velocity(self.vx, self.vy, self.vz, self.yaw_rate)
            time.sleep(0.1)

    # ===================== GAMEPAD & FILES =====================
    # (Mantenha aqui os métodos de save/load mission e gamepad mapping do seu código original)
    # Apenas certifique-se de chamar self.call_js ao invés de usar window diretamente
    
    def save_mission(self):
        if not self.window: return
        file_path = self.window.create_file_dialog(webview.SAVE_DIALOG, save_filename='missao.xml', file_types=('XML (*.xml)',))
        if file_path:
            # Chama função JS para obter o conteúdo e salvar
            self.call_js(f'initiateSave({json.dumps(file_path[0])})')

    def write_mission_data(self, data):
        try:
            with open(data['path'], 'w', encoding='utf-8') as f:
                f.write(data['content'])
            self.call_js('toast("Sucesso", "Missão salva com sucesso")') # Assumindo que vc tem um toast no JS
        except Exception as e:
            print(f"Erro save: {e}")

    def load_mission(self):
        if not self.window: return
        file_paths = self.window.create_file_dialog(webview.OPEN_DIALOG, file_types=('XML (*.xml)',))
        if file_paths:
            try:
                with open(file_paths[0], 'r', encoding='utf-8') as f:
                    content = f.read()
                self.call_js(f'loadWorkspaceXml({json.dumps(content)})')
            except Exception as e:
                print(f"Erro load: {e}")

    def save_gamepad_mapping(self, mapping):
        with open(GAMEPAD_CONFIG_FILE, "w") as f:
            json.dump(mapping, f, indent=2)
        self.gamepad_mapping = mapping

    def load_gamepad_mapping(self):
        if GAMEPAD_CONFIG_FILE.exists():
            with open(GAMEPAD_CONFIG_FILE) as f:
                self.gamepad_mapping = json.load(f)
        else:
            self.gamepad_mapping = self.default_gamepad_mapping.copy()
        self.call_js(f'populateMappingUi({json.dumps(self.gamepad_mapping)})')
        
    def restore_default_mapping(self):
        self.gamepad_mapping = self.default_gamepad_mapping.copy()
        self.save_gamepad_mapping(self.gamepad_mapping)
        self.call_js(f'populateMappingUi({json.dumps(self.gamepad_mapping)})')

    def _gamepad_loop(self):
        # Implementação simplificada do loop original
        DEADZONE = 0.1
        MAX_AXIS = 32767
        while not self.stop_event.is_set():
            try:
                events = inputs.get_gamepad()
                for e in events:
                    # Lógica de mapeamento aqui (copiar do original)
                    pass 
            except:
                time.sleep(0.1)

    def _command_socket_loop(self):
        # Lógica do socket original
        while not self.stop_event.is_set():
            # ... (código de conexão socket aqui)
            time.sleep(1)


# --- STARTUP ---
def start_server():
    """Roda o servidor FastAPI em background"""
    uvicorn.run(app, host="127.0.0.1", port=SERVER_PORT, log_level="error")

if __name__ == '__main__':
    # 1. Inicia o Backend Controller
    backend = DroneBackendController()
    api_bridge = ApiFacade(backend)

    # 2. Inicia o Servidor FastAPI em uma thread separada
    server_thread = threading.Thread(target=start_server, daemon=True)
    server_thread.start()

    # Aguarda o servidor subir (pequeno delay)
    time.sleep(1)

    # 3. Cria a janela apontando para o servidor local
    window = webview.create_window(
        "Estação de Controle Drone",
        f"http://127.0.0.1:{SERVER_PORT}",  # Aponta para o FastAPI
        js_api=api_bridge,                  # Injeta a ponte JS
        width=1280,
        height=800,
        min_size=(1024, 600)
    )
    
    backend.set_window(window)

    # 4. Inicia o PyWebview
    webview.start(backend.start_background_tasks, debug=True)
    
    # 5. Limpeza ao fechar
    backend.shutdown()