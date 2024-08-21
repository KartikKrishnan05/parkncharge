import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.publisher_ = self.create_publisher(Pose, 'start_position', 10)
        self.destination_publisher_ = self.create_publisher(Pose, 'destination_position', 10)
        self.timer = self.create_timer(1.0, self.publish_positions)

    def publish_positions(self):
        start_pose = Pose()
        start_pose.position.x = 0.0
        start_pose.position.y = 0.0
        start_pose.position.z = 0.0

        destination_pose = Pose()
        destination_pose.position.x = 10.0
        destination_pose.position.y = 5.0
        destination_pose.position.z = 0.0

        self.publisher_.publish(start_pose)
        self.destination_publisher_.publish(destination_pose)

        self.get_logger().info(f'Published start position: {start_pose}')
        self.get_logger().info(f'Published destination position: {destination_pose}')

def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
