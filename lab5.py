import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from turtlesim.msg import Pose as TurtlePose
import math

class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtle_control')
        self.init_variables()
        self.init_publisher()
        self.init_subscribers()

    def init_variables(self):
        self.x = 8.0
        self.y = 8.0
        self.theta = 0.0
        self.x_goal = 8.0
        self.y_goal = 8.0

    def init_publisher(self):
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.pub_callback)

    def init_subscribers(self):
        self.pose_subscription = self.create_subscription(
            TurtlePose, '/turtle1/pose', self.pose_callback, 10)
        self.goal_subscription = self.create_subscription(
            Pose2D, '/goal', self.goal_callback, 10)

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

        x_error = self.x_goal - self.x
        y_error = self.y_goal - self.y

        desired_theta = math.atan2(y_error, x_error)
        alpha = desired_theta - self.theta

        if alpha > math.pi:
            alpha -= 2 * math.pi
        elif alpha < -math.pi:
            alpha += 2 * math.pi

        k_alpha = 10
        angular_velocity = k_alpha * alpha

        angle_threshold = 0.05
        if abs(alpha) < angle_threshold:
            angular_velocity = 0.0

        velocity_command = Twist()
        velocity_command.angular.z = angular_velocity
        self.velocity_publisher.publish(velocity_command)

    def goal_callback(self, msg):
        self.x_goal = msg.x
        self.y_goal = msg.y

    def pub_callback(self):
        x_error = self.x_goal - self.x
        y_error = self.y_goal - self.y

        rho = math.sqrt(x_error**2 + y_error**2)

        k_rho = 10
        linear_velocity = k_rho * rho

        distance_threshold = 0.1
        if rho < distance_threshold:
            linear_velocity = 0.0

        velocity_command = Twist()
        velocity_command.linear.x = linear_velocity
        self.velocity_publisher.publish(velocity_command)

def main(args=None):
    rclpy.init(args=args)
    turtle_control_node = TurtleControl()
    rclpy.spin(turtle_control_node)
    turtle_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
