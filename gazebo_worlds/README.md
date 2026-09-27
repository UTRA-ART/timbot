# Gazebo Worlds Folder Overview

The `gazebo_worlds` folder contains simulation environments and assets for testing and developing the robot in Gazebo. It enables safe, repeatable testing and development before deploying to real hardware.

## Structure

- **launch/**: Launch files for starting Gazebo with specific worlds (e.g., `gazebo.launch.py`). These files set up the simulation environment, spawn the robot model, and configure simulation parameters.
- **worlds/**: World files defining different simulation scenarios (e.g., `track.world`, `ramp_track.world`, `empty.world`). These files describe the environment, obstacles, and terrain for the robot to navigate.

## Integration Role

- Provides a variety of simulation environments for testing navigation, perception, and control algorithms.
- Launch files integrate the robot description and bring up the simulation with the appropriate world and robot model.
- Enables system-level testing of all subsystems in a controlled, reproducible environment.
- Supports rapid iteration and debugging by allowing developers to test changes in simulation before deploying to hardware.

## How It Works

1. **Environment Setup**: Launch files start Gazebo with a selected world and spawn the robot model using the description package.
2. **Simulation Execution**: All core subsystems (navigation, perception, control) are brought up in the simulated environment, allowing for end-to-end testing.
3. **Scenario Variation**: Multiple world files allow for testing under different conditions, such as various track layouts and obstacle configurations.

## Usage

- Use the provided launch files to start simulation scenarios:

```bash
ros2 launch gazebo_worlds gazebo.launch.py
```

- Select different world files to test specific scenarios or challenges.

## Relationships

- Depends on the `description` folder for the robot model.
- Integrates with the `launch` folder for system-wide simulation startup.
- Used by navigation and perception packages for development and testing.

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
