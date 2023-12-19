import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Pose2D, Quaternion
from nav_msgs.msg import Odometry
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.init_variables()
        self.init_publishers()
        self.init_subscribers()

    def init_variables(self):
        self.current_pose = Pose()
        self.current_goal = Pose()
        self.distance_threshold = 0.05  # Adjust as needed
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def init_publishers(self):
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(Pose2D, '/goal', 10)

    def init_subscribers(self):
        self.pose_subscription = self.create_subscription(
            Odometry, '/odom', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

        x_error = self.current_goal.position.x - self.current_pose.position.x
        y_error = self.current_goal.position.y - self.current_pose.position.y
        rho = math.sqrt(x_error**2 + y_error**2)
        k_rho = 0.2  # Adjust as needed
        linear_velocity = k_rho * rho

        if rho < self.distance_threshold:
            linear_velocity = 0.0

        desired_orientation = math.atan2(y_error, x_error)
        alpha = desired_orientation - self.quaternion_to_yaw(self.current_pose.orientation)

        if alpha > math.pi:
            alpha -= 2 * math.pi
        elif alpha < -math.pi:
            alpha += 2 * math.pi

        k_alpha = 1  # Adjust as needed
        angular_velocity = k_alpha * alpha

        if abs(alpha) < 0.2:
            angular_velocity = 0.0

        if abs(alpha) >= 0.2:
            velocity_command = Twist()
            velocity_command.angular.z = angular_velocity
            self.velocity_publisher.publish(velocity_command)
        else:
            velocity_command = Twist()
            velocity_command.linear.x = linear_velocity
            self.velocity_publisher.publish(velocity_command)

        self.get_logger().info(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")
        self.get_logger().info(f"Current Pose: x={self.current_pose.position.x}, y={self.current_pose.position.y}")
        self.get_logger().info(f"Goal Pose: x={self.current_goal.position.x}, y={self.current_goal.position.y}")

    def quaternion_to_yaw(self, quaternion):
        return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                          1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2))

    def publish_new_goal(self, x, y):
        goal_msg = Pose()
        goal_msg.position.x = x
        goal_msg.position.y = y
        self.current_goal = goal_msg
        self.get_logger().info(f'New goal published: {goal_msg.position.x}, {goal_msg.position.y}')

    def update_velocity(self):
        linear_velocity = 0.8  # Adjust as needed
        angular_velocity = 0.4  # Adjust as needed

        velocity_command = Twist()
        velocity_command.linear.x = linear_velocity
        velocity_command.angular.z = angular_velocity
        self.velocity_publisher.publish(velocity_command)

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    controller.publish_new_goal(1.5, 1.6)  # Set the initial goal
    controller.spin()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
