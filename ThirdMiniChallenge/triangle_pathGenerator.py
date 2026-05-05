# Imports
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32


class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        #Publisher
        self.goal_pub = self.create_publisher(Pose, 'goal', 10)

        #Subscribers
        self.goal_reached_sub= self.create_subscription(
            Int32,
            'goal_reached',
            self.goal_reached_callback,
            10
        )

        #Timer
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        #states
        # 0 = idle
        # 1 = publishing goals
        # 2 = End
        self.state = 0
	
	#Goals
        self.goals = [
                (2.0, 0.0, 0.0),
                (2.0, 2.0, math.pi / 2.0),
                (0.0, 2.0, math.pi),
                (0.0, 0.0, -math.pi / 2.0)
        ]

        self.current_goal_idx = 0

        self.start_path()

        self.get_logger().info('Path Generator listo. Publicando objetivos secu>

    def start_path(self):
        if len(self.goals) < 3:
            self.get_logger().info('Se requieren al menos 3 posiciones objetivo>
            self.state = 2
            return

        self.state = 1
        self.current_goal_idx = 0

    def goal_reached_callback(self, msg):
        if self.state != 1:
            return

        if msg.data != 1:
            return

        self.get_logger().info(
            f'Objetivo {self.current_goal_idx + 1} alcanzado por el controlador>
        )

        self.current_goal_idx += 1
	
	if self.current_goal_idx >= len(self.goals):
            self.state = 2
            self.get_logger().info('Todos los objetivos fueron completados.')
            return

    def timer_callback(self):
        if self.state != 1:
            return

        if self.current_goal_idx >= len(self.goals):
            return

        self.publish_current_goal()

    def publish_current_goal(self):
        goal_x, goal_y, goal_theta = self.goals[self.current_goal_idx]

        msg = Pose()
        msg.position.x = goal_x
        msg.position.y = goal_y
        msg.position.z = 0.0

        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(goal_theta / 2.0)
        msg.orientation.w = math.cos(goal_theta / 2.0)

        self.goal_pub.publish(msg)

        self.get_logger().info(
            f'Publicando objetivo {self.current_goal_idx + 1}: '
            f'x = {goal_x:.2f}, y = {goal_y:.2f}, theta = {goal_theta:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
