import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from ackermann_msgs.msg import AckermannDriveStamped

class AckermannController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        self.subscription = self.create_subscription(PoseArray, 'trajectory', self.trajectory_callback, 10)
        self.cmd_publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        self.timer = self.create_timer(0.1, self.publish_commands)
        self.trajectory = None
        self.current_index = 0

    def trajectory_callback(self, msg):
        self.trajectory = msg.poses
        self.current_index = 0
        self.get_logger().info(f'Received trajectory with {len(msg.poses)} poses')

    def publish_commands(self):
        if self.trajectory is None or self.current_index >= len(self.trajectory):
            return

        current_pose = self.trajectory[self.current_index]
        self.current_index += 1

        command = AckermannDriveStamped()
        command.drive.speed = 1.0
        command.drive.steering_angle = 0.0  # Simplified, assuming straight line for now

        self.cmd_publisher.publish(command)
        self.get_logger().info(f'Published command: speed={command.drive.speed}, steering_angle={command.drive.steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    ackermann_controller = AckermannController()
    rclpy.spin(ackermann_controller)
    ackermann_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
