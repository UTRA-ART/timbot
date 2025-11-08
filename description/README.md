# Robot Description

This directory contains robot description files (URDF/xacro) that define the robot's physical structure.

## Purpose

Robot description files define the robot's kinematic structure, visual appearance, collision geometry, and sensor placements.

## File Types

- `.urdf` files - Unified Robot Description Format
- `.xacro` files - XML Macros for URDF (allows modular robot descriptions)
- `.yaml` files - Configuration parameters
- `.launch.py` files - Launch files for robot state publisher

## Usage

Launch robot description:

```bash
ros2 launch robot_state_publisher robot_state_publisher.launch.py
```

## Common Contents

- Base link and chassis definition
- Wheel descriptions
- Sensor mountings
- Joint definitions
- Visual and collision meshes
