# ParkNCharge
# Gazebo:
The groundadded.sdf world file currently contains a Pickup-Truck, a white-plane as the inductive loading plate and a ground plate for the entities to stand on.
The origin of these files (excluding white_plane) is on gitlab, gazebo models.
# ROS2 Ackermann Steering Car Trajectory Brnanch

Just the basic Structure of ROS2 Nodes
- NOT connected to Gazebo in any way 
- NO proper PATH planning 
- NO correct Moving of the Car
- Hardcoded inputs for Positions  

## Overview

This project consists of a ROS2-based simulation for an Ackermann steering car in a Gazebo environment. The project includes three main nodes:

1. **Position Publisher**: Publishes the starting and destination positions of the car.
2. **Trajectory Planner**: Receives the positions and calculates a straight-line trajectory between them.
3. **Ackermann Controller**: Converts the trajectory into velocity and steering commands for the Ackermann steering car.
4. **Launching Gazebo Fortress Model**: "export LIBGL_ALWAYS_SOFTWARE=1" is needed to be able to launch the world simulation without crash.
5. **Launching Model in general**: To be able to load the simulation, your Gazebo-Mode-Path has to somewhat look like this: /home/#User#/GAZ/models/gazebo_models

## Nodes

### 1. Position Publisher

This node publishes the starting position and destination position of the car to specified topics.

- **Topic Published**:
  - `/start_position` (geometry_msgs/Pose)
  - `/destination_position` (geometry_msgs/Pose)

### 2. Trajectory Planner

This node subscribes to the start and destination positions, calculates a straight-line trajectory, and publishes the trajectory.

- **Topics Subscribed**:
  - `/start_position` (geometry_msgs/Pose)
  - `/destination_position` (geometry_msgs/Pose)
- **Topic Published**:
  - `/trajectory` (geometry_msgs/PoseArray)

### 3. Ackermann Controller

This node subscribes to the trajectory, converts it into velocity and steering commands, and publishes these commands.

- **Topic Subscribed**:
  - `/trajectory` (geometry_msgs/PoseArray)
- **Topic Published**:
  - `/ackermann_cmd` (ackermann_msgs/AckermannDriveStamped)

#

