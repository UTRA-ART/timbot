# Raspberry Pi Packages

This directory contains ROS2 packages specific to Raspberry Pi hardware and interfaces.

## Purpose

Raspberry Pi packages handle hardware-specific functionality, GPIO control, and Pi-specific peripherals.

## Creating Packages

To create a new Raspberry Pi package:

```bash
cd rpi
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy std_msgs
```

## Common Package Types

- GPIO control nodes
- Pi Camera interfaces
- Hardware monitoring
- Power management
- LED/display control
