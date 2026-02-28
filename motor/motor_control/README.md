# Motor Control Packages

This directory contains ROS2 packages for motor control and actuation.

## Purpose

Motor control packages handle low-level motor control, velocity commands, and actuator interfaces.

## Creating Packages

To create a new motor control package:

```bash
cd motor-control
ros2 pkg create --build-type ament_cmake <package_name> --dependencies rclcpp std_msgs control_msgs
```

## Common Package Types

- Motor driver interfaces
- PID controllers
- Velocity controllers
- Differential drive controllers
- Servo control
