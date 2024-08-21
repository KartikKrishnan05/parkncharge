#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class TeleopTwistKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        self.speed = 0.5
        self.turn = 1.0
        self.key_bindings = {
            'i': (1, 0, 0, 0),
            'o': (1, 0, 0, -1),
            'j': (0, 0, 0, 1),
            'l': (0, 0, 0, -1),
            'u': (1, 0, 0, 1),
            ',': (-1, 0, 0, 0),
            '.': (-1, 0, 0, 1),
            'm': (-1, 0, 0, -1),
            'O': (1, -1, 0, 0),
            'I': (1, 0, 0, 0),
            'J': (0, 1, 0, 0),
            'L': (0, -1, 0, 0),
            'U': (1, 1, 0, 0),
            '<': (-1, 0, 0, 0),
            '>': (-1, -1, 0, 0),
            'M': (-1, 1, 0, 0),
            't': (0, 0, 1, 0),
            'b': (0, 0, -1, 0),
            'k': (0, 0, 0, 0),  
        }
        self.speed_bindings = {
            'q': (1.1, 1.1),
            'z': (0.9, 0.9),
            'w': (1.1, 1.0),
            'x': (0.9, 1.0),
            'e': (1.0, 1.1),
            'c': (1.0, 0.9),
        }
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            print(self.get_instructions())
            while rclpy.ok():
                key = self.get_key()

                if key in self.key_bindings.keys():
                    x, y, z, th = self.key_bindings[key]
                elif key in self.speed_bindings.keys():
                    self.speed = self.speed * self.speed_bindings[key][0]
                    self.turn = self.turn * self.speed_bindings[key][1]
                    print(self.get_status())
                    continue
                else:
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                    if key == '\x03':  # CTRL-C to quit
                        break

                # Only publish when a valid key is pressed (no continuous publishing)
                if key in self.key_bindings.keys():
                    twist = Twist()
                    twist.linear.x = x * self.speed
                    twist.linear.y = y * self.speed
                    twist.linear.z = z * self.speed
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = th * self.turn
                    self.publisher_.publish(twist)
        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_instructions(self):
        return """
        Reading from the keyboard  and Publishing to Twist only when a key is pressed!
        ---------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .

        For Holonomic mode (strafing), hold down the shift key:
        ---------------------------
           U    I    O
           J    K    L
           M    <    >

        t : up (+z)
        b : down (-z)
        k : stop

        anything else : stop

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%

        CTRL-C to quit
        """

    def get_status(self):
        return f"currently:\tspeed {self.speed}\tturn {self.turn}"

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
