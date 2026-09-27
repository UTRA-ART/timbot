# Navigation Folder Overview

The `nav` folder contains all packages related to the robot’s autonomous navigation capabilities. It integrates perception, planning, and control to enable the robot to move safely and efficiently in its environment.

## Structure

- **filter_lidar_data/**: Processes raw LiDAR data to filter noise and extract useful features for localization and obstacle avoidance. This package typically subscribes to raw sensor topics, applies filtering algorithms, and republishes cleaned data for downstream use.
- **load_waypoints/**: Manages loading and handling of waypoint data for path following and mission planning. It provides services or topics for other nodes to access mission waypoints, and may support dynamic waypoint updates. It handles loading in waypoints, allow for the rover to respawn to a different waypoint in sim, and handles detecting and navigating ramps.
- **nav_stack/**: Implements the core navigation stack, including taking info from Odom, path planning, and motion control. This package is responsible for integrating sensor data, generating paths, and sending velocity commands to the robot’s actuators.

## Integration Role

- The navigation packages work together to provide a robust navigation pipeline:
	- LiDAR data is filtered and processed for use in localization and mapping.
	- Waypoints are loaded and managed for mission execution.
	- The navigation stack consumes sensor data and waypoints to plan and execute safe paths.
- These packages interface with other subsystems (e.g., motor control, perception) via ROS2 topics and services, enabling coordinated autonomous operation.
- The navigation system is designed to be modular, allowing for the integration of additional sensors or planning algorithms as needed.

## How It Works

1. **Sensor Processing**: `filter_lidar_data` cleans and preprocesses LiDAR data, making it suitable for use in localization and obstacle detection.
2. **Waypoint Management**: `load_waypoints` loads mission waypoints from files or external sources and provides them to the navigation stack.
3. **Path Planning and Control**: `nav_stack` uses filtered sensor data and waypoints to localize the robot, plan paths, and generate control commands.

## Usage

- Each package in this folder can be launched individually or as part of a system-wide launch file (see the `launch` folder).
- Configuration files for each package allow for tuning of algorithms and parameters.

## Relationships

- Consumes perception data from the `cv` folder (e.g., lane detection) and sensor data from hardware drivers.
- Sends velocity and control commands to the motor control subsystem.
- Provides navigation outputs to higher-level mission planners or user interfaces.
- Obstacle avoidance
- Nav2 configuration and launch files
- Costmap layers
- Behavior trees
