import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class PositionLogger(Node):

    def __init__(self):
        super().__init__('position_logger')

        # Suscripción al tópico
        self.subscription = self.create_subscription(
            String,
            '/current_cartesian_position',
            self.position_callback,
            10
        )

        # Archivo donde se guardarán los datos
        self.file = open("posiciones_robot.csv", "w")
        self.file.write("timestamp,x,y,z,rx,ry,rz\n")  # encabezado CSV

        self.start_time = time.time()
        self.duration = 120  # segundos = 2 minutos
        self.get_logger().info("📡 Registrando posiciones por 2 minutos...")

    def position_callback(self, msg):
        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            self.get_logger().info("✅ Registro finalizado. Archivo guardado en posiciones_robot.csv")
            self.file.close()
            rclpy.shutdown()
            return

        try:
            # Mensaje tipo string → "x,y,z,rx,ry,rz"
            parts = [float(v) for v in msg.data.strip().split(',')]
            if len(parts) < 6:
                raise ValueError("Formato inesperado en el tópico.")

            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            line = f"{timestamp},{parts[0]},{parts[1]},{parts[2]},{parts[3]},{parts[4]},{parts[5]}\n"
            self.file.write(line)

        except Exception as e:  
            self.get_logger().error(f"Error al parsear posición: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PositionLogger()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()

