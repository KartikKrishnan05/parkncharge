import subprocess
import time

class BridgeManager:
    def __init__(self):
        self.bridges = []

    def add_bridge(self, ros2_topic, ros2_msg_type, ignition_msg_type, direction="both"):
        """
        Add a new bridge with the specified topics and message types.
        Parameters:
            ros2_topic: ROS 2 topic name.
            ros2_msg_type: ROS 2 message type.
            ignition_msg_type: Ignition message type.
            direction: Bridge direction ("ros2_to_ign" for ROS to Ignition, "ign_to_ros2" for Ignition to ROS, or "both").
        """
        if direction == "ros2_to_ign":
            bridge_command = [
                "ros2", "run", "ros_gz_bridge", "parameter_bridge", 
                f"{ros2_topic}@{ros2_msg_type}>ignition.msgs.{ignition_msg_type}"
            ]
        elif direction == "ign_to_ros2":
            bridge_command = [
                "ros2", "run", "ros_gz_bridge", "parameter_bridge", 
                f"{ros2_topic}@{ros2_msg_type}[ignition.msgs.{ignition_msg_type}"
            ]
        else:
            bridge_command = [
                "ros2", "run", "ros_gz_bridge", "parameter_bridge", 
                f"{ros2_topic}@{ros2_msg_type}@ignition.msgs.{ignition_msg_type}"
            ]
        self.bridges.append(bridge_command)
    
    def start_bridges(self):
        """
        Start all added bridges in separate subprocesses.
        """
        for bridge_command in self.bridges:
            print(f"Starting bridge: {' '.join(bridge_command)}")
            subprocess.Popen(bridge_command)

    def stop_all_bridges(self):
        """
        Stops all running bridges. Note: You'll need to manually kill subprocesses if needed.
        """
        print("Stopping all bridges. Make sure to manually kill subprocesses if needed.")


# Example usage of the BridgeManager
if __name__ == "__main__":
    bridge_manager = BridgeManager()

    # Add the odometry bridge (Ignition to ROS 2)
    bridge_manager.add_bridge(
        ros2_topic="/vehicle_blue/odom", 
        ros2_msg_type="nav_msgs/msg/Odometry", 
        ignition_msg_type="Odometry",
        direction="ign_to_ros2"  # Ignition to ROS 2 bridge
    )

    # Add the cmd_vel bridge for velocity control (ROS 2 to Ignition)
    bridge_manager.add_bridge(
        ros2_topic="/model/vehicle_blue/cmd_vel", 
        ros2_msg_type="geometry_msgs/msg/Twist", 
        ignition_msg_type="Twist",
        direction="ros2_to_ign"  # ROS 2 to Ignition bridge
    )

    # Start all bridges
    bridge_manager.start_bridges()

    # Keep the script alive to ensure bridges remain active
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down bridges.")
        bridge_manager.stop_all_bridges()
