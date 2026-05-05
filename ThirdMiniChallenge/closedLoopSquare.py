# Imports
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy


class ClosedLoopSquareCtrl(Node):
    def __init__(self):
        super().__init__('closed_loop_square_ctrl')

        #QoS for encoder topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        #Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Int32, 'goal_reached', 10)

        #Subscribers
        self.vel_enc_l_sub = self.create_subscription(
            Float32,
            'VelocityEncL',
            self.vel_enc_l_callback,
            qos
        )
        self.vel_enc_r_sub = self.create_subscription(
            Float32,
            'VelocityEncR',
            self.vel_enc_r_callback,
            qos
        )
        self.start_sub = self.create_subscription(
            Int32,
            'figure_cmd',
            self.start_callback,
            10
        )
        self.goal_sub = self.create_subscription(
            Pose,
            'goal',
            self.goal_callback,
            10
        )

        #Timer
        self.timer_period = 0.05   #20 Hz same frequency as when we made our fi>
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        #Current pose estimated from encoders
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_ready = False

        #Current wheel speeds
        self.vel_enc_l = 0.0
        self.vel_enc_r = 0.0
        self.left_ready = False
        self.right_ready = False
	
	self.active = False
        self.enabled = False

        #states
        # 0 = idle
        # 1 = move to waypoint
        # 2 = align final heading
        self.state = 0

        #Current goal
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0
        self.goal_ready = False

        #Goal comparison tolerances
        self.goal_pos_epsilon = 0.01
        self.goal_ang_epsilon = 0.02

        #Position and angle tolerances
        self.pos_tolerance = 0.02
        self.ang_tolerance = 0.04

        #Kp gain
        self.kp_dist = 0.25
        self.kp_ang = 1.5
        self.kp_final_ang = 2.5

        #Ki gain
        self.ki_dist = 0.01
        self.ki_ang = 0.05

	#Kd gain
        self.kd_ang = 0.1
        self.kd_dist = 0.2

        #Previous error
        self.prev_dist_error = 0.0
        self.prev_ang_error = 0.0
        self.int_dist_error = 0.0
        self.int_ang_error = 0.0

        #System limits
        self.max_linear = 0.2
        self.max_angular = 0.5

        #Puzzlebot parameters
        self.r = 0.0525
        self.l = 0.085

        #Simple filter for encoder velocities
        self.alpha = 0.8

        self.get_logger().info('Nodo de lazo cerrado listo. Esperando comando e>

    def start_callback(self, msg):
        data = msg.data

        if not self.odom_ready:
            self.get_logger().info('Aun no recibo velocidades de encoders para >
            return

        if data != 1:
            self.get_logger().info('Para esta entrega base usa data = 1 para ha>
            return
	self.enabled = True
        self.active = False
        self.goal_ready = False
        self.state = 0
        self.reset_pid_terms()

        self.get_logger().info('Controlador habilitado.')

    def goal_callback(self, msg):
        if not self.enabled:
            self.get_logger().info('Objetivo recibido pero el controlador aun n>
            return

        new_goal_x = msg.position.x
        new_goal_y = msg.position.y
        new_goal_theta = self.quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        same_goal = (
            self.goal_ready
            and abs(new_goal_x - self.goal_x) < self.goal_pos_epsilon
            and abs(new_goal_y - self.goal_y) < self.goal_pos_epsilon
            and abs(self.wrap_to_pi(new_goal_theta - self.goal_theta)) < self.g>
        )

	if same_goal:
            return

        self.goal_x = new_goal_x
        self.goal_y = new_goal_y
        self.goal_theta = new_goal_theta

        self.goal_ready = True
        self.active = True
        self.state = 1

        self.reset_pid_terms()

        self.get_logger().info(
            f'Nuevo objetivo recibido: x = {self.goal_x:.2f}, y = {self.goal_y:>
        )

    def vel_enc_l_callback(self, msg):
        self.vel_enc_l = self.alpha * self.vel_enc_l + (1.0 - self.alpha) * msg>
        self.left_ready = True
        self.update_encoder_ready()

    def vel_enc_r_callback(self, msg):
        self.vel_enc_r = self.alpha * self.vel_enc_r + (1.0 - self.alpha) * msg>
        self.right_ready = True
        self.update_encoder_ready()

    def update_encoder_ready(self):
        if self.left_ready and self.right_ready:
            self.odom_ready = True
    
    def update_pose_from_encoders(self):
        #Differential drive kinematics
        v = (self.r / 2.0) * (self.vel_enc_r + self.vel_enc_l)
        w = (self.r / (2.0 * self.l)) * (self.vel_enc_r - self.vel_enc_l)

        #Pose integration
        self.x += v * math.cos(self.theta) * self.timer_period
        self.y += v * math.sin(self.theta) * self.timer_period
        self.theta += w * self.timer_period
        self.theta = self.wrap_to_pi(self.theta)

    def control_loop(self):
        cmd = Twist()

        if self.odom_ready:
            self.update_pose_from_encoders()

        if self.state == 0:
            self.cmd_vel_pub.publish(cmd)
            return

        if self.state == 1:
            goal_x = self.goal_x
            goal_y = self.goal_y
            goal_theta = self.goal_theta

            dx = goal_x - self.x
            dy = goal_y - self.y

            dist_error = math.sqrt(dx**2 + dy**2)
            desired_heading = math.atan2(dy, dx)
            ang_error = self.wrap_to_pi(desired_heading - self.theta)

	    #PID terms
            self.int_dist_error += dist_error * self.timer_period
            d_dist_error = (dist_error - self.prev_dist_error) / self.timer_per>

            self.int_ang_error += ang_error * self.timer_period
            d_ang_error = (ang_error - self.prev_ang_error) / self.timer_period

            #Base control
            v = (
                self.kp_dist * dist_error
                + self.ki_dist * self.int_dist_error
                + self.kd_dist * d_dist_error
            )

            w = (
                self.kp_ang * ang_error
                + self.ki_ang * self.int_ang_error
                + self.kd_ang * d_ang_error
            )

            # If heading error is large, reduce forward motion
            if abs(ang_error) > 0.35:
                v = 0.0

            # Saturation
            v = self.clamp(v, -self.max_linear, self.max_linear)
            w = self.clamp(w, -self.max_angular, self.max_angular)

            cmd.linear.x = v
            cmd.angular.z = w

            self.prev_dist_error = dist_error
            self.prev_ang_error = ang_error

	    if dist_error <= self.pos_tolerance:
                self.state = 2
                self.reset_pid_terms()
                self.get_logger().info('Objetivo alcanzado en posicion. Alinean>

        elif self.state == 2:
            goal_theta = self.goal_theta

            final_ang_error = self.wrap_to_pi(goal_theta - self.theta)

            self.int_ang_error += final_ang_error * self.timer_period
            d_ang_error = (final_ang_error - self.prev_ang_error) / self.timer_>

            w = (
                self.kp_final_ang * final_ang_error
                + self.ki_ang * self.int_ang_error
                + self.kd_ang * d_ang_error
            )

            w = self.clamp(w, -self.max_angular, self.max_angular)

            cmd.linear.x = 0.0
            cmd.angular.z = w

            self.prev_ang_error = final_ang_error

            if abs(final_ang_error) <= self.ang_tolerance:
                self.get_logger().info('Objetivo completado.')

                reached_msg = Int32()
                reached_msg.data = 1
                self.goal_reached_pub.publish(reached_msg)
		
		self.active = False
                self.goal_ready = False
                self.state = 0
                self.reset_pid_terms()

        self.cmd_vel_pub.publish(cmd)

    def reset_pid_terms(self):
        self.prev_dist_error = 0.0
        self.prev_ang_error = 0.0
        self.int_dist_error = 0.0
        self.int_ang_error = 0.0

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_to_pi(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopSquareCtrl()

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
