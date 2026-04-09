# Imports
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')

        # Publisher a /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber al topico que define la figura
        # data:
        # 1 = cuadrado
        # 2 = rectangulo
        # 3 = triangulo
        # 4 = trapecio
        # 5 = pentagono
        # 6 = hexagono
        # 7 = heptagono
        # 8 = octagono
        self.figure_sub = self.create_subscription(
            Int32,
            'figure_cmd',
            self.figure_callback,
            10
        )

        # Velocidades
        self.linear_speed = 0.2    # m/s  -> velocidad maxima pedida
        self.angular_speed = 0.6   # rad/s -> giro sobre su propio eje

        # Timer de control
        self.timer_period = 0.05   # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Estados
        # 0 = idle
        # 1 = forward
        # 2 = stop_after_forward
        # 3 = rotate
        # 4 = stop_after_rotate
        # 5 = finished
        self.state = 0
        self.state_start_time = self.get_clock().now()

        # Variables de la figura actual
        self.figure_active = False
        self.current_step = 0
        self.side_distances = []
        self.turn_angles_deg = []

        # Tiempos de pausa
        self.stop_time = 0.4

        self.get_logger().info('Nodo listo. Esperando un mensaje en /figure_cmd ...')

    def figure_callback(self, msg):
        data = msg.data

        # Si ya esta ejecutando una figura, ignora nuevos comandos
        if self.figure_active:
            self.get_logger().info('Ya estoy dibujando una figura. Comando ignorado.')
            return

        plan_loaded = self.load_figure_plan(data)

        if not plan_loaded:
            self.get_logger().info(f'Valor no valido: {data}. Usa un entero del 1 al 8.')
            return

        self.figure_active = True
        self.current_step = 0
        self.state = 1
        self.state_start_time = self.get_clock().now()

        self.get_logger().info(f'Figura recibida con data = {data}. Iniciando trayectoria...')

    def load_figure_plan(self, data):
        # Aqui defines las distancias de cada lado (en metros)
        # y los angulos de giro (en grados) despues de cada lado.

        # Figuras regulares:
        # angulo exterior = 360 / numero_de_lados

        if data == 1:
            # Cuadrado
            side = 0.5
            self.side_distances = [side, side, side, side]
            self.turn_angles_deg = [90.0, 90.0, 90.0, 90.0]
            return True

        elif data == 2:
            # Rectangulo
            long_side = 0.8
            short_side = 0.4
            self.side_distances = [long_side, short_side, long_side, short_side]
            self.turn_angles_deg = [90.0, 90.0, 90.0, 90.0]
            return True

        elif data == 3:
            # Triangulo equilatero
            sides = 3
            side = 0.5
            angle = 360.0 / sides
            self.side_distances = [side] * sides
            self.turn_angles_deg = [angle] * sides
            return True

        elif data == 4:
            # Trapecio isosceles aproximado
            # Puedes cambiar estas medidas si quieres otro trapecio
            base_mayor = 0.8
            lado = 0.4
            base_menor = 0.4

            # Secuencia de lados:
            # base mayor -> lado inclinado -> base menor -> lado inclinado
            self.side_distances = [base_mayor, lado, base_menor, lado]

            # Angulos exteriores aproximados
            # Ajustables segun el trapecio que quieras
            self.turn_angles_deg = [120.0, 60.0, 120.0, 60.0]
            return True

        elif data == 5:
            # Pentagono regular
            sides = 5
            side = 0.4
            angle = 360.0 / sides
            self.side_distances = [side] * sides
            self.turn_angles_deg = [angle] * sides
            return True

        elif data == 6:
            # Hexagono regular
            sides = 6
            side = 0.35
            angle = 360.0 / sides
            self.side_distances = [side] * sides
            self.turn_angles_deg = [angle] * sides
            return True

        elif data == 7:
            # Heptagono regular
            sides = 7
            side = 0.3
            angle = 360.0 / sides
            self.side_distances = [side] * sides
            self.turn_angles_deg = [angle] * sides
            return True

        elif data == 8:
            # Octagono regular
            sides = 8
            side = 0.25
            angle = 360.0 / sides
            self.side_distances = [side] * sides
            self.turn_angles_deg = [angle] * sides
            return True

        else:
            return False

    def control_loop(self):
        cmd = Twist()
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9

        # Estado idle
        if self.state == 0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        # Si ya termino todos los lados
        if self.current_step >= len(self.side_distances):
            self.state = 5

        # Estado 1: avanzar
        if self.state == 1:
            current_distance = self.side_distances[self.current_step]
            forward_time = current_distance / self.linear_speed

            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

            if elapsed_time >= forward_time:
                self.state = 2
                self.state_start_time = now
                self.get_logger().info(f'Lado {self.current_step + 1} completado. Pausa antes del giro.')

        # Estado 2: detenerse despues de avanzar
        elif self.state == 2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            if elapsed_time >= self.stop_time:
                self.state = 3
                self.state_start_time = now
                angle_deg = self.turn_angles_deg[self.current_step]
                self.get_logger().info(
                    f'Girando {angle_deg:.2f} grados sobre su propio eje...'
                )

        # Estado 3: girar sobre su propio eje
        elif self.state == 3:
            angle_deg = self.turn_angles_deg[self.current_step]
            angle_rad = math.radians(angle_deg)
            rotate_time = angle_rad / self.angular_speed

            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed

            if elapsed_time >= rotate_time:
                self.state = 4
                self.state_start_time = now
                self.get_logger().info(f'Giro {self.current_step + 1} completado.')

        # Estado 4: detenerse despues de girar
        elif self.state == 4:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            if elapsed_time >= self.stop_time:
                self.current_step += 1

                if self.current_step >= len(self.side_distances):
                    self.state = 5
                    self.state_start_time = now
                    self.get_logger().info('Figura terminada.')
                else:
                    self.state = 1
                    self.state_start_time = now
                    self.get_logger().info(f'Iniciando lado {self.current_step + 1}...')

        # Estado 5: terminado
        elif self.state == 5:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.figure_active = False
            self.current_step = 0
            self.side_distances = []
            self.turn_angles_deg = []
            self.state = 0
            self.state_start_time = now

            self.get_logger().info('Regresando a idle. Esperando un nuevo topico...')

        self.cmd_vel_pub.publish(cmd)

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCtrl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()