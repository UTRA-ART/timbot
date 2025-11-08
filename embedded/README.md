# Embedded System Packages

This directory contains ROS2 packages for embedded system integration and microcontroller communication.

## Purpose

Embedded packages handle communication between ROS2 and microcontrollers (e.g., Arduino, STM32) and embedded firmware interfaces.

## Creating Packages

To create a new embedded package:

```bash
cd embedded
ros2 pkg create --build-type ament_cmake <package_name> --dependencies rclcpp std_msgs
```

## Common Package Types

- Serial communication nodes
- Microcontroller interfaces
- Firmware upload utilities
- Low-level sensor interfaces
- Real-time control bridges
