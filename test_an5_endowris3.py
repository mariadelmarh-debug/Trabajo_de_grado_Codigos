#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from frhal_msgs.srv import ROSCmdInterface
from std_msgs.msg import String
import serial
import time
import threading
from math import isclose

# =========================
# CONFIGURACIÓN
# =========================
PAUSA_ROBOT = 10  # ⏸️ Segundos de pausa en P3, P6 y P9
TOL_POS = 1.0     # tolerancia en mm para decir que el robot llegó

# =========================
# Control de Servos Pololu
# =========================
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

def set_target(channel, target):
    target = int(target / 0.25)  # Pololu usa unidades de 0.25 µs
    command = bytearray([0x84, channel, target & 0x7F, (target >> 7) & 0x7F])
    ser.write(command)

def mover_suave(channel, inicio, fin, pasos=80, pausa=0.02):
    step = max(1, int(abs(fin - inicio) / pasos))
    if inicio < fin:
        recorrido = range(inicio, fin + 1, step)
    else:
        recorrido = range(inicio, fin - 1, -step)

    for pos in recorrido:
        set_target(channel, pos)
        time.sleep(pausa)

def mover_servos_pausa():
    """
    Mueve motor 0 y motor 2 en paralelo (ida y vuelta).
    Luego espera hasta completar PAUSA_ROBOT segundos en total.
    """
    pos_inicial = 1500
    pos_45 = pos_inicial + 250   # ≈ 45 grados

    # Llevar motores a posición inicial
    set_target(0, pos_inicial)
    set_target(2, pos_inicial)
    time.sleep(1)

    def ciclo_motor(channel):
        mover_suave(channel, pos_inicial, pos_45, pasos=120, pausa=0.02)
        time.sleep(0.5)
        mover_suave(channel, pos_45, pos_inicial, pasos=120, pausa=0.02)

    inicio = time.time()

    # Motores en paralelo
    t1 = threading.Thread(target=ciclo_motor, args=(0,))
    t2 = threading.Thread(target=ciclo_motor, args=(2,))
    t1.start()
    t2.start()
    t1.join()
    t2.join()

    # Medir duración real
    duracion = time.time() - inicio
    restante = PAUSA_ROBOT - duracion
    if restante > 0:
        time.sleep(restante)


# =========================
# Nodo ROS
# =========================
class RobotTrajectory(Node):

    def __init__(self):
        super().__init__('robot_trajectory')

        self.client = self.create_client(ROSCmdInterface, '/FR_ROS_API_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /FR_ROS_API_service...')

        self.subscription = self.create_subscription(
            String,
            '/current_cartesian_position',
            self.position_callback,
            10
        )

        self.current_position = None
        self.commands = []
        self.targets = []     # posiciones objetivo de la trayectoria
        self.point_id = 1
        self.executing = False
        self.waiting_pause = None

    def position_callback(self, msg):
        raw_data = msg.data.strip()
        try:
            parts = [float(v) for v in raw_data.split(',')]
            if len(parts) < 6:
                return

            self.current_position = {
                'x': parts[0],
                'y': parts[1],
                'z': parts[2],
                'rx': parts[3],
                'ry': parts[4],
                'rz': parts[5]
            }

            # Si estamos esperando una pausa, verificar si ya llegó
            if self.waiting_pause is not None:
                target = self.waiting_pause
                if (isclose(self.current_position['x'], target['x'], abs_tol=TOL_POS) and
                    isclose(self.current_position['y'], target['y'], abs_tol=TOL_POS) and
                    isclose(self.current_position['z'], target['z'], abs_tol=TOL_POS)):
                    
                    self.get_logger().info("✅ Robot llegó al punto de pausa, moviendo servos...")
                    mover_servos_pausa()
                    self.waiting_pause = None
                    self.execute_next_command()

            # Primera vez que recibimos posición → arrancar trayectoria
            if not self.executing:
                self.executing = True
                self.build_trajectory()
                self.execute_next_command()

        except Exception as e:
            self.get_logger().error(f"Error al parsear posición: {e}")

    def build_trajectory(self):
        x = self.current_position['x']
        y = self.current_position['y']
        z = self.current_position['z']
        rx = self.current_position['rx']
        ry = self.current_position['ry']
        rz = self.current_position['rz']

        def add_point(nx, ny, nz, insert_pause=False):
            cmd = f"CARTPoint({self.point_id},{nx:.3f},{ny:.3f},{nz:.3f},{rx:.3f},{ry:.3f},{rz:.3f})"
            move = f"MoveL(CART{self.point_id},10)"
            self.commands.append(cmd)
            self.commands.append(move)

            if insert_pause:
                self.commands.append({"pause_servos": {"x": nx, "y": ny, "z": nz}})

            self.point_id += 1

        # Reset
        self.commands.append("ResetAllError()")
        add_point(x, y, z)  # P1 inicial

        displacements = [
            (0, +10, 0),   # P2
            (0, 0, -10),   # P3 (pausa)
            (0, 0, +10),   # P4
            (0, +10, 0),   # P5
            (0, 0, -10),   # P6 (pausa)
            (0, 0, +10),   # P7
            (0, +10, 0),   # P8
            (0, 0, -10),   # P9 (pausa)
            (0, 0, +10),   # P10
            (0, +10, 0),   # P11
        ]

        pause_points = {1, 4, 7}  # índices en displacements para pausas

        for idx, (dx, dy, dz) in enumerate(displacements):
            x += dx
            y += dy
            z += dz
            add_point(x, y, z, insert_pause=(idx in pause_points))

    def execute_next_command(self):
        if not self.commands:
            self.get_logger().info("✅ Trayectoria finalizada")
            return

        cmd = self.commands.pop(0)

        if isinstance(cmd, dict) and "pause_servos" in cmd:
            self.waiting_pause = cmd["pause_servos"]
            self.get_logger().info(f"⏸️ Esperando llegada a {self.waiting_pause} para mover servos...")
            return

        self.get_logger().info(f"Enviando: {cmd}")
        self.send_request(cmd)

    def send_request(self, command):
        req = ROSCmdInterface.Request()
        req.cmd_str = command
        future = self.client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Respuesta servicio: {response.cmd_res}")
        except Exception as e:
            self.get_logger().error(f"Error en llamada al servicio: {e}")

        # Continuar con el siguiente comando
        if self.waiting_pause is None:
            self.execute_next_command()


def main(args=None):
    rclpy.init(args=args)
    node = RobotTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    ser.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

