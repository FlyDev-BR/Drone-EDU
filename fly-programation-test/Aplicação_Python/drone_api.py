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
        print("🛡️ Armando motores...")
        self.set_mode("GUIDED")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        self.wait_until_armed(True)

    def disarm(self):
        print("🛡️ Desarmando...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        self.wait_until_armed(False)

    def wait_until_armed(self, should_be_armed, timeout=5):
        start = time.time()
        while time.time() - start < timeout:
            if self.is_armed() == should_be_armed:
                print(f"✅ Estado Armado: {should_be_armed}")
                return
            time.sleep(0.2)
        print("⚠️ Timeout aguardando armar/desarmar")

    def takeoff(self, altitude_metros):
        if not self.is_armed():
            self.arm()
        
        print(f"🛫 Decolando para {altitude_metros}m...")
        self.set_mode("GUIDED")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude_metros)
        
        # Espera chegar na altura (margem de 5%)
        self.wait_altitude(altitude_metros, tolerance=0.5)

    def land(self):
        print("🛬 Pousando...")
        self.set_mode("LAND")
        # Loop simples para esperar pouso
        while self.get_altitude() > 0.2:
            time.sleep(0.5)
        print("✅ Pouso detectado.")
        self.disarm()

    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping().get(mode_name.upper())
        if mode_id is None:
            print(f"❌ Modo {mode_name} desconhecido")
            return
        
        # Só envia se não estiver já no modo
        if self.get_mode() != mode_name.upper():
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id, 0, 0, 0, 0, 0)
            time.sleep(0.5) # Dá tempo pro drone processar

    # --- MOVIMENTAÇÃO POR POSIÇÃO (A MELHOR FORMA) ---
    def move_meters(self, north, east, down, abort_event=None):
        """
        Move X metros relativo à posição atual usando SET_POSITION_TARGET_LOCAL_NED.
        Máscara de bits: 0b110111111000 (Usa apenas Posição x,y,z)
        """
        print(f"📍 Movendo Relativo: N={north} E={east} D={down}")
        self.set_mode("GUIDED")

        type_mask = 0b110111111000

        self.master.mav.set_position_target_local_ned_send(
            0, # time_boot_ms
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # Relativo ao corpo do drone (frente/direita)
            type_mask,
            north, east, down, 
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

        distance = math.sqrt(north**2 + east**2 + down**2)
        duration = (distance / 1.0) + 2
        
        self.wait(duration, abort_event)
        print("✅ Movimento finalizado (timeout)")

    # --- GIRO (YAW) ---
    def turn_degrees(self, degrees, speed=45, abort_event=None):
        print(f"🔄 Girando {degrees} graus...")
        self.set_mode("GUIDED")
        
        direction = 1 if degrees > 0 else -1
        abs_angle = abs(degrees)

        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
            abs_angle,
            speed,
            direction,
            1,
            0, 0, 0
        )

        duration = (abs_angle / speed) + 2
        self.wait(duration, abort_event)
        print("✅ Giro finalizado.")

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
            mavutil.mavlink.MAV_COMP_ID_ALL,
            MAV_CMD_SET_GPIO, 0,
            pin,
            state,
            0, 0, 0, 0, 0)
            
        print("Comando de GPIO enviado para todos os componentes.")
        time.sleep(0.5)

    # ===================== UTIL =====================
    def wait(self, seconds, abort_event=None):
        for _ in range(int(seconds * 10)):
            if abort_event and abort_event.is_set():
                raise RuntimeError("MISSION_ABORTED")
            time.sleep(0.1)

    # ===================== TELEMETRIA =====================

    def get_altitude(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if not msg:
            return 0.0
        return msg.relative_alt / 1000.0

    def get_pitch(self):
        msg = self._recv('ATTITUDE')
        return math.degrees(msg.pitch) if msg else 0.0

    def get_roll(self):
        msg = self._recv('ATTITUDE')
        return math.degrees(msg.roll) if msg else 0.0

    def get_yaw(self):
        msg = self._recv('ATTITUDE')
        return math.degrees(msg.yaw) if msg else 0.0

    def get_ground_speed(self):
        msg = self._recv('VFR_HUD')
        return msg.groundspeed if msg else 0.0

    def get_battery_voltage(self):
        msg = self._recv('SYS_STATUS')
        return (msg.voltage_battery / 1000.0) if msg else 0.0

    def get_battery_percentage(self):
        msg = self._recv('SYS_STATUS')
        return msg.battery_remaining if msg else 0
    
    def get_gps_fix(self):
        msg = self._recv('GPS_RAW_INT')
        return msg.fix_type if msg else 0

    def is_armed(self):
        return self.master.motors_armed()

    def get_mode(self):
        return self.master.flightmode

    def _recv(self, msg_type, timeout=0.01):
        try:
            return self.master.recv_match(type=msg_type, blocking=False)
        except:
            return None

    def set_manual_velocity(self, vx, vy, vz, yaw_rate):
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000100111000111,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, math.radians(yaw_rate)
        )

    def wait_altitude(self, target_alt, tolerance=0.3, timeout=10):
        print(f"⏳ Aguardando altitude {target_alt}m...")
        start = time.time()
        while time.time() - start < timeout:
            current = self.get_altitude()
            if abs(current - target_alt) <= tolerance:
                print(f"✅ Altitude atingida: {current:.2f}m")
                return
            time.sleep(0.2)
        print("⚠️ Timeout de altitude (prosseguindo assim mesmo)")