# Gazebo World Files

This directory contains Gazebo simulation world files and launch configurations.

## Purpose

Gazebo worlds define simulation environments for testing the robot in various scenarios.

## File Types

- `.world` files - Gazebo world definitions
- `.launch.py` files - Launch files for simulation
- `.sdf` files - Model definitions
- Configuration files for simulated sensors

## Usage

Launch a simulation world:

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/world/file.world
```

## Common World Types

- Empty worlds for basic testing
- Indoor environments
- Outdoor terrains
- Obstacle courses
- Competition arenas
