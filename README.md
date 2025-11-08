# Timbot

A ROS2-based autonomous robot project developed by UTRA-ART.

## Overview

Timbot is a robotics platform featuring computer vision, odometry, navigation, and motor control capabilities. The project is organized into modular ROS2 packages for easy development and maintenance.

## Getting Started

### WSL Setup

If you're using Windows Subsystem for Linux (WSL), please refer to the **setup.pdf** file in this repository for detailed instructions on setting up your WSL environment for ROS2 development.

### Contributing

Before contributing to this project, please read the **[CONTRIBUTING.md](CONTRIBUTING.md)** file. It contains essential information about:

- Setting up ROS2 packages in the workspace
- Creating new packages in directories like `odom`, `cv`, `nav`, etc.
- Building and testing your code
- Branching practices and workflow
- Pull request guidelines

## Project Structure

```
timbot/
├── cv/              # Computer vision packages
├── odom/            # Odometry packages
├── nav/             # Navigation packages
├── sensor-drivers/  # Sensor driver packages
├── motor-control/   # Motor control packages
├── embedded/        # Embedded system packages
├── rpi/             # Raspberry Pi specific packages
├── gazebo-worlds/   # Simulation world files
└── description/     # Robot description (URDF/xacro)
```

## Quick Start

1. Follow the WSL setup instructions in `setup.pdf` (if applicable)
2. Read the [CONTRIBUTING.md](CONTRIBUTING.md) file for development setup
3. Install ROS2 dependencies
4. Build the workspace with `colcon build`
5. Start developing!

## License

See [LICENSE](LICENSE) file for details.

## Contact

For questions or issues, please open an issue on GitHub or contact the UTRA-ART team.

