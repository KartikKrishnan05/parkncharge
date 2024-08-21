import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')
        self.subscription_start = self.create_subscription(Pose, 'start_position', self.start_callback, 10)
        self.subscription_dest = self.create_subscription(Pose, 'destination_position', self.dest_callback, 10)
        self.trajectory_publisher = self.create_publisher(PoseArray, 'trajectory', 10)
        self.start_pose = None
        self.destination_pose = None

    def start_callback(self, msg):
        self.start_pose = msg
        self.get_logger().info(f'Received start position: {msg}')
        self.plan_trajectory()

    def dest_callback(self, msg):
        self.destination_pose = msg
        self.get_logger().info(f'Received destination position: {msg}')
        self.plan_trajectory()

    def plan_trajectory(self):
        if self.start_pose is None or self.destination_pose is None:
            return

        trajectory = PoseArray()
        step = 0.1
        dx = self.destination_pose.position.x - self.start_pose.position.x
        dy = self.destination_pose.position.y - self.start_pose.position.y
        distance = (dx ** 2 + dy ** 2) ** 0.5
        steps = int(distance / step)

        for i in range(steps + 1):
            pose = Pose()
            pose.position.x = self.start_pose.position.x + (dx / steps) * i
            pose.position.y = self.start_pose.position.y + (dy / steps) * i
            pose.position.z = 0.0
            trajectory.poses.append(pose)

        self.trajectory_publisher.publish(trajectory)
        self.get_logger().info(f'Published trajectory with {len(trajectory.poses)} poses')

def main(args=None):
    rclpy.init(args=args)
    trajectory_planner = TrajectoryPlanner()
    rclpy.spin(trajectory_planner)
    trajectory_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
