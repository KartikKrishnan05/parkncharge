import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class VehicleBluePositionPrinter(Node):
    def __init__(self):
        super().__init__('vehicle_blue_position_printer')

        # Subscribe to the /vehicle_blue/odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/vehicle_blue/odom',
            self.odom_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        # Extract the position from the odometry message
        position = msg.pose.pose.position

        # Print the vehicle's position
        self.get_logger().info(f"Vehicle Blue Position: x={position.x}, y={position.y}, z={position.z}")

        # After printing the position, shut down the node
        self.get_logger().info("Position printed. Shutting down...")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    vehicle_blue_position_printer = VehicleBluePositionPrinter()

    # Spin the node to process the callback
    try:
        rclpy.spin(vehicle_blue_position_printer)
    except KeyboardInterrupt:
        pass

    # Destroy the node after completion
    vehicle_blue_position_printer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
