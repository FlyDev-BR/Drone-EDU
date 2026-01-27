# drone_api.py

from pymavlink import mavutil
import time
import math

# RASPBERRY_PI_IP = "192.168.4.1"
RASPBERRY_PI_IP = "127.0.0.1"
MAVLINK_TCP_PORT = "5760"
CONNECTION_STRING = f'tcp:{RASPBERRY_PI_IP}:{MAVLINK_TCP_PORT}'

class DroneAPI:
    def __init__(self, connect_str):
        print(f"Tentando conexão TCP com o drone em {connect_str}...")
        self.master = mavutil.mavlink_connection(connect_str)
        print("Aguardando heartbeat do drone...")
        self.master.wait_heartbeat()
        print("Heartbeat recebido! Drone conectado e pronto.")

    def arm(self):
        """Arma os motores do drone."""
        print("Armando o drone...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        print("Comando de armar enviado.")
        time.sleep(1)

    def disarm(self):
        """Desarma os motores do drone."""
        print("Desarmando o drone...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        print("Comando de desarmar enviado.")

    def takeoff(self, altitude_metros):
        """Decola para uma altitude específica."""
        print(f"Decolando para {altitude_metros} metros...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude_metros)
        print("Comando de decolagem enviado.")

    def land(self):
        """Pousa o drone na localização atual."""
        print("Pousando...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
        print("Comando de pouso enviado.")

    def set_manual_velocity(self, vx, vy, vz, yaw_rate_deg_s):
        """Define uma velocidade contínua para o controle manual."""
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000100111000111,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, math.radians(yaw_rate_deg_s))

    def set_mode(self, mode_name):
        """Define o modo de voo do drone."""
        print(f"Tentando definir o modo para {mode_name}...")
        mode_id = self.master.mode_mapping().get(mode_name.upper())
        if mode_id is None:
            print(f"ERRO: Modo '{mode_name}' não é válido.")
            return
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0)
        print(f"Comando para definir modo {mode_name} enviado.")

    def wait(self, seconds):
        """Pausa a execução do script por um número de segundos."""
        print(f"Aguardando por {seconds} segundos...")
        time.sleep(seconds)
        print("Espera finalizada.")

    def turn_degrees(self, degrees, speed=30):
        """Gira o drone em seu próprio eixo (yaw) em modo de missão (ex: GUIDED)."""
        print(f"Iniciando giro de {degrees}°...")
        direction = 1 if degrees > 0 else -1
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
            abs(degrees), speed, direction, 1, 0, 0, 0)
        wait_time = (abs(degrees) / speed) + 1.0
        print(f"Aguardando aproximadamente {wait_time:.2f}s para o giro completar.")
        time.sleep(wait_time)
        print("Giro concluído.")

    def move_meters(self, north_m, east_m, down_m):
        """Move o drone uma distância específica em metros (para missões em bloco)."""
        INDOOR_SPEED_MPS = 0.3
        distance = math.sqrt(north_m**2 + east_m**2 + down_m**2)
        if distance == 0:
            print("Nenhuma distância para mover.")
            return
        duration = distance / INDOOR_SPEED_MPS
        velocity_x = (north_m / distance) * INDOOR_SPEED_MPS
        velocity_y = (east_m / distance) * INDOOR_SPEED_MPS
        velocity_z = (down_m / distance) * INDOOR_SPEED_MPS
        print(f"Movendo ({north_m}m N, {east_m}m E, {down_m}m D) por {duration:.2f}s.")
        msg = self.master.mav.set_position_target_local_ned_encode(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b0000111111000111,
            0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0)
        for _ in range(int(duration * 5)):
            self.master.mav.send(msg)
            time.sleep(0.2)
        self.stop()
        print("Movimento por metros concluído.")

    def stop(self):
        """Envia um comando para parar todo o movimento (usado internamente por move_meters)."""
        msg = self.master.mav.set_position_target_local_ned_encode(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b0000111111000111,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        for _ in range(5):
            self.master.mav.send(msg)
            time.sleep(0.1)

    def calibrate_level(self):
        """Envia o comando para calibrar o nível do acelerômetro."""
        print("Enviando comando de calibração de nível...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 0, 0, 0, 0, 1, 0, 0)
        print("Comando enviado. Mantenha o drone perfeitamente parado e nivelado.")

    def reboot_autopilot(self):
        """Envia o comando para reiniciar o controlador de voo."""
        print("Enviando comando de reboot do autopiloto...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 1, 0, 0, 0, 0, 0, 0)
        print("Comando de reboot enviado. A conexão será perdida.")
        
    def set_gpio(self, pin, state):
        """Envia um comando customizado para controlar um pino GPIO no companion computer."""
        MAV_CMD_SET_GPIO = 31010

        print(f"Enviando comando para setar GPIO {pin} para {'HIGH' if state == 1 else 'LOW'}...")
        
        self.master.mav.command_long_send(
            self.master.target_system, 
            mavutil.mavlink.MAV_COMP_ID_ALL, # Envia para TODOS os componentes
            MAV_CMD_SET_GPIO, 0,
            pin,      # param1: O número do pino
            state,    # param2: O estado (0 ou 1)
            0, 0, 0, 0, 0)
            
        print("Comando de GPIO enviado para todos os componentes.")
        time.sleep(0.5)
