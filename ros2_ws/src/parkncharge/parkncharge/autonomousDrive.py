import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class MoveToGoal(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('move_to_goal')

        # Initialize goal position and starting position
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.start_x = 0.0
        self.start_y = 0.0
        self.tolerance = 0.5  # Tolerance to stop near the goal
        self.path = []  # List to store the trajectory

        # Current position and orientation of the car
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation_z = 0.0

        # Flag to indicate whether the trajectory is initialized
        self.trajectory_initialized = False

        # Create a publisher for the /model/vehicle_blue/cmd_vel topic (to send velocity commands)
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)

        # Subscribe to the /vehicle_blue/odom topic (to get the car's current position)
        self.subscription = self.create_subscription(
            Odometry,
            '/vehicle_blue/odom',
            self.odom_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Timer to keep moving the car along the pre-calculated path
        self.timer = self.create_timer(0.1, self.follow_trajectory)

    def odom_callback(self, msg):
        # Update the car's current position and orientation from the odometry data
        position = msg.pose.pose.position
        self.current_x = position.x
        self.current_y = position.y

        # Extract the car's orientation in quaternion format (not directly used here)
        orientation = msg.pose.pose.orientation
        self.current_orientation_z = orientation.z  # Simplified 2D motion

        # Only initialize the trajectory once, after the first odometry update
        if not self.trajectory_initialized:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.calculate_trajectory()  # Calculate the path to the goal
            self.trajectory_initialized = True

    def calculate_trajectory(self):
        # This function calculates a simple straight-line path from the start to the goal.
        num_points = 100  # Number of points on the path

        for i in range(num_points + 1):
            t = i / num_points  # Parameter that varies from 0 to 1
            x = (1 - t) * self.start_x + t * self.goal_x
            y = (1 - t) * self.start_y + t * self.goal_y
            self.path.append((x, y))  # Append each point on the path

        self.get_logger().info(f"Trajectory calculated with {len(self.path)} points.")

    def follow_trajectory(self):
        if len(self.path) == 0:
            return  # No trajectory to follow yet

        # Get the next waypoint in the trajectory
        target_x, target_y = self.path.pop(0)

        # Calculate the distance to the next waypoint
        distance_to_waypoint = math.sqrt((target_x - self.current_x) ** 2 + (target_y - self.current_y) ** 2)

        if distance_to_waypoint <= self.tolerance:
            if len(self.path) == 0:
                # Stop when reaching the final goal
                self.stop_vehicle()
                self.get_logger().info(f"Reached the goal position in new Way: x={self.goal_x}, y={self.goal_y}")
                self.timer.cancel()  # Stop the timer
                return

        # Calculate the angle to the next waypoint
        angle_to_waypoint = math.atan2(target_y - self.current_y, target_x - self.current_x)

        # Create a Twist message to move the car
        move_cmd = Twist()

        # Move forward with a constant speed
        move_cmd.linear.x = 1.0

        # Adjust the angular velocity to steer towards the target
        # Simple proportional control to turn towards the waypoint
        move_cmd.angular.z = angle_to_waypoint

        # Publish the movement command
        self.publisher_.publish(move_cmd)

        self.get_logger().info(f"Moving towards waypoint: x={target_x}, y={target_y} (distance: {distance_to_waypoint:.2f})")

    def stop_vehicle(self):
        # Send a stop command by setting both linear and angular velocities to zero
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.publisher_.publish(stop_cmd)
        self.get_logger().info("Vehicle stopped.")


def main(args=None):
    rclpy.init(args=args)

    # Ask the user for the target position
    goal_x = float(input("Enter the target x position: "))
    goal_y = float(input("Enter the target y position: "))

    # Create and run the node
    move_to_goal = MoveToGoal(goal_x, goal_y)

    try:
        rclpy.spin(move_to_goal)
    except KeyboardInterrupt:
        pass

    # Cleanup the node
    move_to_goal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
