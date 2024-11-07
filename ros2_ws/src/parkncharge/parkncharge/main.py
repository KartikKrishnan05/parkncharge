import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

class AckermannPathPlanner:
    def __init__(self, start, goal, max_steering_angle=0.5, turn_radius=1.0):
        self.start = start
        self.goal = goal
        self.max_steering_angle = max_steering_angle
        self.turn_radius = turn_radius
        self.waypoints = []

    def generate_waypoints(self):
        # Generate the initial smooth waypoints to the goal
        dx = (self.goal[0] - self.start[0]) / 40  # Divide into 40 segments
        dy = (self.goal[1] - self.start[1]) / 40
        
        for i in range(41):
            x = self.start[0] + i * dx
            y = self.start[1] + i * dy
            self.waypoints.append((x, y))
        
    def detect_and_adjust_turns(self):
        adjusted_waypoints = [self.waypoints[0]]  # Start with the first waypoint
        
        for i in range(1, len(self.waypoints) - 1):
            prev_wp = adjusted_waypoints[-1]
            current_wp = self.waypoints[i]
            next_wp = self.waypoints[i + 1]

            # Calculate direction vectors
            vec1 = (current_wp[0] - prev_wp[0], current_wp[1] - prev_wp[1])
            vec2 = (next_wp[0] - current_wp[0], next_wp[1] - current_wp[1])

            # Calculate the angle between the vectors
            angle = self.angle_between_vectors(vec1, vec2)

            # If the angle is close to 90 degrees, add intermediate points for a 90° turn
            if 80 <= abs(angle) <= 100:
                # Add a point for a right-angle turn approximation
                self.add_turn_waypoints(adjusted_waypoints, prev_wp, current_wp, vec1)
            else:
                adjusted_waypoints.append(current_wp)
        
        # Add the final goal point
        adjusted_waypoints.append(self.goal)
        
        self.waypoints = adjusted_waypoints

    def angle_between_vectors(self, v1, v2):
        # Calculate the angle in degrees between vectors v1 and v2
        dot_product = v1[0] * v2[0] + v1[1] * v2[1]
        mag_v1 = math.sqrt(v1[0]**2 + v1[1]**2)
        mag_v2 = math.sqrt(v2[0]**2 + v2[1]**2)
        angle_rad = math.acos(dot_product / (mag_v1 * mag_v2))
        return math.degrees(angle_rad)

    def add_turn_waypoints(self, adjusted_waypoints, prev_wp, current_wp, vec1):
        # Calculate turn waypoints based on Ackermann steering constraints
        # For now, add a simple approximation of a 90-degree turn
        mid_x = (prev_wp[0] + current_wp[0]) / 2 + self.turn_radius * vec1[1]
        mid_y = (prev_wp[1] + current_wp[1]) / 2 - self.turn_radius * vec1[0]
        adjusted_waypoints.append((mid_x, mid_y))
        adjusted_waypoints.append(current_wp)


class MoveToGoalAckermann(Node):
    def __init__(self):
        super().__init__('move_to_goal_ackermann')
        self.goal_position = (10.0, -2.0)  # Example goal
        self.current_position = (0.0, 0.0)
        self.path_planner = AckermannPathPlanner(start=self.current_position, goal=self.goal_position)
        self.path_planner.generate_waypoints()
        self.path_planner.detect_and_adjust_turns()
        
        self.odom_subscriber = self.create_subscription(Odometry, '/vehicle_blue/odom', self.odom_callback, 10)
        self.cmd_publisher = self.create_publisher(Twist, '/vehicle_blue/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.follow_path)
        
        self.current_waypoint_index = 0
        self.print_planned_path()
    
    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def follow_path(self):
        if self.current_waypoint_index >= len(self.path_planner.waypoints):
            self.get_logger().info("Reached the goal!")
            self.stop_robot()
            return
        
        target = self.path_planner.waypoints[self.current_waypoint_index]
        distance = math.sqrt((target[0] - self.current_position[0])**2 + (target[1] - self.current_position[1])**2)
        
        if distance < 0.5:
            self.current_waypoint_index += 1
        else:
            self.move_towards(target)

    def move_towards(self, target):
        move_cmd = Twist()
        dx = target[0] - self.current_position[0]
        dy = target[1] - self.current_position[1]
        distance = math.sqrt(dx**2 + dy**2)

        move_cmd.linear.x = min(distance, 1.0)  # Cap speed at 1.0
        move_cmd.angular.z = math.atan2(dy, dx)
        
        self.cmd_publisher.publish(move_cmd)
    
    def stop_robot(self):
        stop_cmd = Twist()
        self.cmd_publisher.publish(stop_cmd)
    
    def print_planned_path(self):
        self.get_logger().info("Planned path with Ackermann constraints:")
        for wp in self.path_planner.waypoints:
            self.get_logger().info(f"Waypoint: x={wp[0]:.2f}, y={wp[1]:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
