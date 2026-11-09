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

## File explanation
- `track.world`: Standard map, has walls so LIDAR marks lane lines 
- `track_cv.world`: Standard map, has no walls so CV marks lane lines
- `ramp_track.world`: Test with ramp in front of robot on spawn
- `track_old.world`: Before we adjusted the track (see below)
- `empty.world`: Just the floor, with no obstacles

- `IGVC_pavement.png`: Modified map to widen areas which are clearly wrong
- `IGVC_pavement_old.png`: Map of the obstacle course from the IGVC rules
- `composite_preview.png`: A composite of the two above images for comparison