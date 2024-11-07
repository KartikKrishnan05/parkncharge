#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import time
import math  # Import math module for calculations

class TeleopTwistKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        self.speed = 0.5
        self.turn = 1.0
        self.move_duration = 0.5  # Move for half a second before stopping
        self.settings = termios.tcgetattr(sys.stdin)

        # Key bindings for movement
        self.key_bindings = {
            'i': (1, 0, 0, 0),
            'o': (1, 0, 0, -1),
            'u': (1, 0, 0, 1),
            ',': (-1, 0, 0, 0),
            '.': (-1, 0, 0, 1),
            'm': (-1, 0, 0, -1),
            'O': (1, -1, 0, 0),
            'I': (1, 0, 0, 0),
            'U': (1, 1, 0, 0),
            '<': (-1, 0, 0, 0),
            '>': (-1, -1, 0, 0),
            'M': (-1, 1, 0, 0),
            't': (0, 0, 1, 0),
            'b': (0, 0, -1, 0),
            'k': (0, 0, 0, 0),  
        }

        # Speed adjustment bindings
        self.speed_bindings = {
            'q': (1.1, 1.1),
            'z': (0.9, 0.9),
            'w': (1.1, 1.0),
            'x': (0.9, 1.0),
            'e': (1.0, 1.1),
            'c': (1.0, 0.9),
        }

    def get_key(self):
        """Capture keypress from the terminal."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        """Main loop to capture keypresses and publish Twist messages."""
        try:
            print(self.get_instructions())  # Display instructions
            while rclpy.ok():
                key = self.get_key()

                if key in self.key_bindings.keys() or key in ['j', 'l']:
                    if key == 'l':
                        # Turn 90 degrees to the right
                        self.turn_90_degrees('right')
                    elif key == 'j':
                        # Turn 90 degrees to the left
                        self.turn_90_degrees('left')
                    else:
                        # Regular movement
                        x, y, z, th = self.key_bindings.get(key, (0, 0, 0, 0))
                        twist = Twist()
                        twist.linear.x = x * self.speed
                        twist.linear.y = y * self.speed
                        twist.linear.z = z * self.speed
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = th * self.turn
                        self.publisher_.publish(twist)

                        # Sleep for a short duration (move for a short period)
                        time.sleep(self.move_duration)

                        # Immediately stop the robot after the short movement
                        self.stop_robot()
                elif key in self.speed_bindings.keys():
                    self.speed = self.speed * self.speed_bindings[key][0]
                    self.turn = self.turn * self.speed_bindings[key][1]
                    print(self.get_status())
                    continue
                else:
                    if key == '\x03':  # CTRL-C to quit
                        break
        finally:
            self.stop_robot()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def turn_90_degrees(self, direction):
        """Rotate the car approximately 90 degrees in place to the specified direction."""
        # Parameters for the turn
        for x in range(9):
            forward_speed = 0.2    # Slow forward speed
            turn_speed = 1.0       # Angular speed for steering
            move_duration = 3.0    # Duration to achieve approximately 90-degree turn

            if direction == 'right':
                angular_z = -abs(turn_speed)
            elif direction == 'left':
                angular_z = abs(turn_speed)
            else:
                self.get_logger().error("Invalid direction for turn_90_degrees")
                return

            # Create Twist message for turning
            twist = Twist()
            twist.linear.x = forward_speed
            twist.angular.z = angular_z

            # Start the turn
            self.publisher_.publish(twist)
            time.sleep(move_duration)

            # Stop the car after turning
            self.stop_robot()

    def stop_robot(self):
        """Send a zero velocity command to stop the robot."""
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)

    def get_instructions(self):
        """Display teleoperation instructions."""
        return """
        Reading from the keyboard and Publishing to Twist!
        -------------------------------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .

        j : Turn 90 degrees to the left
        l : Turn 90 degrees to the right

        t : up (+z)
        b : down (-z)
        k : stop

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%

        CTRL-C to quit
        """

    def get_status(self):
        """Display current speed and turn values."""
        return f"currently:\tspeed {self.speed}\tturn {self.turn}"

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
