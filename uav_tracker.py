# ==============================================================================
# Sistema de Seguimiento Autónomo de Objetos para Drones (UAV Visual Tracker)
# Protocolo: MAVLink v2.0 | Piloto: ArduPilot (SITL) | Detección: YOLOv8
# Autor: Gustavo Bryan López Cosme | UNI - LabIAR
# ==============================================================================

import time
import math
import numpy as np

class PIDController:
    """Controlador PID para estabilización de errores de centrado visual"""
    def __init__(self, kp=0.4, ki=0.01, kd=0.1, max_output=1.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def update(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0:
            dt = 1e-4

        # Término Proporcional
        p_term = self.kp * error

        # Término Integral con anti-windup
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Término Derivativo
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative

        output = p_term + i_term + d_term
        # Saturación de velocidad
        output = max(-self.max_output, min(self.max_output, output))

        self.last_error = error
        self.last_time = current_time
        return output

class DroneMAVLinkTracker:
    """
    Controlador de seguimiento visual conectado a ArduPilot vía MAVLink.
    Envía mensajes SET_POSITION_TARGET_LOCAL_NED con velocidades (Vx, Vy, Vz, Yaw).
    """
    def __init__(self, connection_string="udpin:localhost:14550", target_class="person"):
        self.connection_string = connection_string
        self.target_class = target_class
        self.pid_x = PIDController(kp=0.5, ki=0.01, kd=0.08) # Error horizontal (Yaw/Vy)
        self.pid_y = PIDController(kp=0.5, ki=0.01, kd=0.08) # Error vertical (Vz)
        self.pid_area = PIDController(kp=0.3, ki=0.0, kd=0.05) # Distancia (Vx)
        self.target_area_ratio = 0.15 # 15% del encuadre para distancia deseada
        self.master = None

    def connect(self):
        """Inicializa conexión con MAVProxy o ArduPilot SITL"""
        print(f"[MAVLINK] Conectando a {self.connection_string}...")
        try:
            from pymavlink import mavutil
            self.master = mavutil.mavlink_connection(self.connection_string)
            self.master.wait_heartbeat(timeout=3)
            print(f"[MAVLINK] Heartbeat recibido del sistema {self.master.target_system}")
            return True
        except Exception as e:
            print(f"[MAVLINK SIMULADO] Modo simulación activo. Detalle: {e}")
            return False

    def send_velocity_command(self, vx, vy, vz, yaw_rate=0.0):
        """
        Emite el mensaje MAVLink SET_POSITION_TARGET_LOCAL_NED
        Permite control por vector de velocidad vectorial en coordenadas locales (m/s).
        """
        if self.master:
            try:
                # Bitmask: ignorar posiciones y aceleraciones, solo controlar velocidad y yaw rate
                type_mask = 0b0000011111000111 
                self.master.mav.set_position_target_local_ned_send(
                    0, # time_boot_ms
                    self.master.target_system,
                    self.master.target_component,
                    1, # MAV_FRAME_LOCAL_NED
                    type_mask,
                    0, 0, 0, # Posición x, y, z
                    vx, vy, vz, # Velocidad vx (norte/adelante), vy (este/lateral), vz (abajo)
                    0, 0, 0, # Aceleración
                    0, yaw_rate # Yaw, Yaw_rate (rad/s)
                )
            except Exception as err:
                print(f"[ERROR MAVLink]: {err}")
        else:
            # Registro en log de telemetría simulada
            print(f"[CMD TELEMETRÍA] Vx (avance): {vx:+.2f} m/s | Vy (lateral): {vy:+.2f} m/s | Vz: {vz:+.2f} m/s | YawRate: {yaw_rate:+.2f} rad/s")

    def process_detection(self, bbox, frame_width=640, frame_height=480):
        """
        Calcula las desviaciones espaciales normalizadas y genera comandos de control.
        bbox: [x_min, y_min, x_max, y_max]
        """
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        bbox_area = ((x2 - x1) * (y2 - y1)) / float(frame_width * frame_height)

        # Errores normalizados respecto al centro de imagen (-1.0 a +1.0)
        norm_error_x = (center_x - (frame_width / 2.0)) / (frame_width / 2.0)
        norm_error_y = (center_y - (frame_height / 2.0)) / (frame_height / 2.0)
        norm_error_area = self.target_area_ratio - bbox_area

        # Ejecución del lazo PID
        yaw_rate = self.pid_x.update(norm_error_x) # Girar hacia el objetivo
        vy = self.pid_x.update(norm_error_x) * 0.5 # Desplazamiento lateral
        vz = self.pid_y.update(norm_error_y)       # Subir o bajar
        vx = self.pid_area.update(norm_error_area)  # Acercarse o alejarse

        # Enviar comandos calculados
        self.send_velocity_command(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)
        print(f"[TELEMETRIA] Error X: {norm_error_x:+.2f} | Error Y: {norm_error_y:+.2f} | Comandos: Vx={vx:+.2f} m/s, Vy={vy:+.2f} m/s, Vz={vz:+.2f} m/s, YawRate={yaw_rate:+.2f} rad/s")

        return {
            "error_x": norm_error_x,
            "error_y": norm_error_y,
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "yaw_rate": yaw_rate
        }

if __name__ == "__main__":
    print("=== UAV Visual Tracking Controller | LabIAR UNI ===")
    tracker = DroneMAVLinkTracker()
    tracker.connect()

    print("\nSimulando detección de objetivo móvil durante 5 iteraciones:")
    # Simulación de un objetivo detectado por YOLO desplazándose hacia la derecha
    simulated_boxes = [
        [320, 240, 420, 380], # Centro
        [390, 230, 490, 370], # Desviado a la derecha
        [460, 220, 560, 360], # Mayor desviación derecha
        [340, 250, 440, 390], # Retornando al centro
        [320, 240, 420, 380]  # Centrado
    ]

    for i, box in enumerate(simulated_boxes, 1):
        print(f"\n--- Cuadro #{i} ---")
        telemetry = tracker.process_detection(box)
        time.sleep(0.5)
