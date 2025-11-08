# Sensor Driver Packages

This directory contains ROS2 packages for interfacing with various sensors.

## Purpose

Sensor driver packages handle communication with hardware sensors and publish sensor data to ROS2 topics.

## Creating Packages

To create a new sensor driver package:

```bash
cd sensor-drivers
ros2 pkg create --build-type ament_cmake <package_name> --dependencies rclcpp sensor_msgs std_msgs
```

## Common Package Types

- LiDAR drivers
- IMU drivers
- Ultrasonic sensors
- GPS modules
- Time-of-flight sensors
