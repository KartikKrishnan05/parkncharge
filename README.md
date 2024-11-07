
# ParkNCharge ROS2 & Gazebo Simulation Setup

## Table of Contents
- [ParkNCharge ROS2 \& Gazebo Simulation Setup](#parkncharge-ros2--gazebo-simulation-setup)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Setup Instructions](#setup-instructions)
    - [Docker Setup](#docker-setup)
    - [Building the Environment](#building-the-environment)
  - [Running the Simulation](#running-the-simulation)
    - [Starting the Docker Environment](#starting-the-docker-environment)
    - [Stopping the Simulation](#stopping-the-simulation)
  - [Files Overview](#files-overview)
    - [Dockerfile](#dockerfile)
    - [groundadded.sdf](#groundaddedsdf)
    - [tmux Configuration](#tmux-configuration)
  - [Node Descriptions](#node-descriptions)
    - [1. `MoveAndMonitor.py` (MoveToGoalAckermann)](#1-moveandmonitorpy-movetogoalackermann)
    - [2. `read_odomentry.py` (VehicleBluePositionPrinter)](#2-read_odomentrypy-vehiclebluepositionprinter)
    - [3. `gazebo_ros_launch.py` (MoveVehicleBlue)](#3-gazebo_ros_launchpy-movevehicleblue)
    - [4. `controller.py` (TeleopTwistKeyboard)](#4-controllerpy-teleoptwistkeyboard)

---

## Overview
This project sets up a Gazebo simulation world and a ROS2 environment for vehicle control and visualization. The setup includes Docker containers, `tmux` for session management, and Gazebo plugins to simulate physics and sensor data for vehicles in the world.

## Setup Instructions

### Docker Setup
The Docker image is based on ROS2 Humble and includes necessary tools for development and simulation using Gazebo.

1. Clone the repository and navigate to the project directory:

    ```bash
    git clone https://github.com/your-repo/parkncharge.git
    cd parkncharge
    ```

2. **Build the Docker environment:**
   To build the Docker environment, use `docker-compose` with the following commands:

    ```bash
    sudo service docker start
    docker compose up --build
    ```

### Building the Environment
Once the Docker environment is set up, open a new terminal and execute the following command to access the running container:

```bash
docker exec -it rosgazebo-container /bin/bash
```

From within the container, navigate to your ROS2 workspace and build the project:

```bash
cd parkncharge/ros2_ws
colcon build
source install/setup.bash
```

This will compile the ROS2 packages and set up the environment.

## Running the Simulation

### Starting the Docker Environment

1. **In the first terminal window**:
   Start Docker and the Docker container:
   ```bash
   sudo service docker start
   docker compose up --build
   ```

2. **In the second terminal window**:
   Navigate into the project directory:
   ```bash
   cd parkncharge
   ```
   Then start the `tmux` session with the following command:
   ```bash
   tmuxinator start --project-config=$(pwd)/tmux.yml
   ```

This will automatically launch all necessary processes in `tmux`, including running Gazebo, ROS bridges, and vehicle control scripts.

### Stopping the Simulation

1. **To switch between panes** in `tmux` (e.g., to stop individual processes), press `Ctrl + b` followed by `o`.
   
2. **To stop individual processes**, press `Ctrl + c` within the desired pane.
   
3. **To stop all processes** and exit the `tmux` session, use the following command:
   ```bash
   tmux kill-server
   ```

---

## Files Overview

### Dockerfile
The `Dockerfile` sets up the environment by:
- Using the ROS2 Humble desktop image.
- Configuring the proxy for network settings.
- Installing necessary dependencies, ROS2, Gazebo, and additional packages.
- Creating a user for the Docker container and configuring the environment.

This file automates the setup of the ROS2 and Gazebo environment, ensuring all necessary tools are installed and configured.

### groundadded.sdf
The `groundadded.sdf` file defines the world and models for the Gazebo simulation. It includes:
- Physics configurations.
- Plugins for sensors and scene broadcasting.
- GUI elements to visualize the world in Gazebo.
- Models for the ground plane, vehicle, and additional elements.

The `groundadded.sdf` file is the core of the Gazebo simulation, where the vehicle is loaded, and physics are applied.

### tmux Configuration
The `tmux` configuration file (`tmux.yml`) manages multiple panes in a `tmux` session for running various components of the simulation. It includes:
- A pane for running Gazebo with the `groundadded.sdf` world.
- Panes for running ROS2 and Gazebo bridges for topics like vehicle control (`/cmd_vel`) and odometry (`/odom`).
- A pane for building and running the ROS2 package that controls and monitors the vehicle.

To start the `tmux` session, use the command:

---

## Node Descriptions

### 1. `MoveAndMonitor.py` (MoveToGoalAckermann)
This node controls the movement of the vehicle towards a predefined goal. It subscribes to:
- `/vehicle_blue/odom` for the vehicle's current position and orientation.
- `/world/visualize_lidar_world/pose/info` to get the target goal positions.

It calculates the distance and heading to the goal and publishes velocity commands to `/model/vehicle_blue/cmd_vel` to move the vehicle towards it. The node stops once the goal is reached.

### 2. `read_odomentry.py` (VehicleBluePositionPrinter)
This node simply listens to the `/vehicle_blue/odom` topic and prints the current position of the vehicle to the console. After printing the position, the node shuts down.

### 3. `gazebo_ros_launch.py` (MoveVehicleBlue)
This node is responsible for moving the vehicle in a predefined pattern. It publishes velocity commands to `/model/vehicle_blue/cmd_vel` to make the vehicle move forward, turn left, and stop at specific intervals. The movement is programmed directly in the initialization of the node.

### 4. `controller.py` (TeleopTwistKeyboard)
This node allows for manual control of the vehicle using the keyboard. It reads keypresses from the user and publishes corresponding velocity commands to `/model/vehicle_blue/cmd_vel` to move the vehicle forward, backward, and turn left or right. The speed and turning can be adjusted in real time using the keyboard.

