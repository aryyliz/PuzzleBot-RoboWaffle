# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

#Class Definition
class OpenLoopSquareCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_square_ctrl')

        # Uncomment this function if using Gazebo for simulating the robot: self.wait_for_ros_time()

        # Publisher to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to start the state machine
        self.start_sub = self.create_subscription(
            Float32,
            'start_square',
            self.start_callback,
            10
        )

        # State definitions
        # 0: initial
        # 1: forward
        # 2: stop
        # 3: rotate
        self.state = 0

        # Robot starts inactive
        self.active = False

        # Counter for completed sides
        self.square_count = 0

        # Time-based control variables
        self.state_start_time = self.get_clock().now()

        # Define speeds
        self.linear_speed = 0.2    # m/s
        self.angular_speed = 0.5   # rad/s

        # Define durations
        self.forward_time = 1.2 / self.linear_speed        # Time to move 1.2 m
        self.stop_time = 1.0                               # Stop for 1 second
        self.rotate_time = 1.5708 / self.angular_speed     # Time to rotate 90 deg

        # Timer to update state machine
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Open loop square controller initialized')
        self.get_logger().info('Robot is in INITIAL state, waiting for /start_square topic...')

    def start_callback(self, msg):
        if msg.data > 0.0 and not self.active:
            self.active = True
            self.state = 1
            self.square_count = 0
            self.state_start_time = self.get_clock().now()
            self.get_logger().info('Start signal received. Beginning square trajectory...')

    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9

        cmd = Twist()

        if self.state == 0:
            # INITIAL: robot fully stopped
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            if not self.active:
                self.get_logger().info('INITIAL state: robot stopped.')

        elif self.state == 1:
            # FORWARD: move 2 m
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info('FORWARD state: moving forward...')

            if elapsed_time >= self.forward_time:
                self.state = 2
                self.state_start_time = now
                self.get_logger().info('2 meters completed. Changing to STOP state...')

        elif self.state == 2:
            # STOP: stop robot completely
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('STOP state: robot stopped...')

            if elapsed_time >= self.stop_time:
                self.state = 3
                self.state_start_time = now
                self.get_logger().info('Stop completed. Changing to ROTATE state...')

        elif self.state == 3:
            # ROTATE: rotate right 90 degrees
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            self.get_logger().info('ROTATE state: rotating right 90 degrees...')

            if elapsed_time >= self.rotate_time:
                self.square_count += 1
                self.get_logger().info(f'Side completed: {self.square_count}/4')

                if self.square_count >= 4:
                    self.state = 0
                    self.active = False
                    self.state_start_time = now
                    self.get_logger().info('Square completed. Returning to INITIAL state...')
                else:
                    self.state = 1
                    self.state_start_time = now
                    self.get_logger().info('Starting next side of the square...')

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')


# Main
def main(args=None):
    rclpy.init(args=args)

    node = OpenLoopSquareCtrl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


# Execute Node
if __name__ == '__main__':
    main()
