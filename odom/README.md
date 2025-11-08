# Odometry Packages

This directory contains ROS2 packages related to odometry and localization.

## Purpose

Odometry packages handle position estimation, encoder data processing, and sensor fusion for robot localization.

## Creating Packages

To create a new odometry package:

```bash
cd odom
ros2 pkg create --build-type ament_cmake <package_name> --dependencies rclcpp nav_msgs geometry_msgs tf2 tf2_ros
```

## Common Package Types

- Wheel encoder odometry
- Visual odometry
- IMU integration
- Sensor fusion (e.g., Kalman filters)
