#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveVehicleBlue(Node):

    def __init__(self):
        super().__init__('move_vehicle_blue')

        # Create a publisher for the /model/vehicle_blue/cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)

        # Create the twist messages
        self.move_forward = Twist()
        self.move_forward.linear.x = 1.0
        self.move_forward.angular.z = 0.0

        self.turn_left = Twist()
        self.turn_left.linear.x = 0.0
        self.turn_left.angular.z = 0.5

        self.turn_right = Twist()
        self.turn_right.linear.x = 0.0
        self.turn_right.angular.z = -0.5

        # Move forward 10 times
        for _ in range(10):
            self.publisher_.publish(self.move_forward)
            time.sleep(1)  # Assuming 1 second for each move forward

        # Turn left 5 times
        for _ in range(4):
            self.publisher_.publish(self.turn_left)
            time.sleep(1)  # Assuming 1 second for each turn


        for _ in range(7):
            self.publisher_.publish(self.move_forward)
            time.sleep(1)  # Assuming 1 second for each move forward

        for _ in range(4):
            self.publisher_.publish(self.turn_right)
            time.sleep(1)  # Assuming 1 second for each turn

        for _ in range(7):
            self.publisher_.publish(self.move_forward)
            time.sleep(1)  # Assuming 1 second for each move forward

        # Stop the vehicle
        self.stop_vehicle = Twist()
        self.publisher_.publish(self.stop_vehicle)

        self.get_logger().info("Finished moving vehicle_blue")

def main(args=None):
    rclpy.init(args=args)
    
    move_vehicle_blue = MoveVehicleBlue()

    # Since the movement is done in __init__, just sleep to keep the node alive
    try:
        rclpy.spin(move_vehicle_blue)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    move_vehicle_blue.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
