# Gazebo World Files

This directory contains Gazebo simulation world files and launch configurations.

## Purpose

Gazebo worlds define simulation environments for testing the robot in various scenarios.

## File Types

- `.world` files - Gazebo world definitions
- `.launch.py` files - Launch files for simulation
- `.sdf` files - Model definitions
- Configuration files for simulated sensors

## Environment Paths
set the gazebo environment path to: 
export GZ_SIM_RESOURCE_PATH=~/.gazebo/models:$GZ_SIM_RESOURCE_PATH
## Usage

Launch a simulation world:

```bash
ros2 launch gazebo_worlds load_igvc_full.launch.py
```

## Common World Types

- Empty worlds for basic testing
- Indoor environments
- Outdoor terrains
- Obstacle courses
- Competition arenas
