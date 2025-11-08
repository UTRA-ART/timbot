# Contributing to Timbot

Thank you for your interest in contributing to Timbot! This guide will help you set up your development environment and follow our project conventions.

## Table of Contents
- [Development Setup](#development-setup)
- [ROS2 Package Structure](#ros2-package-structure)
- [Creating ROS2 Packages](#creating-ros2-packages)
- [Building and Testing](#building-and-testing)
- [Branching Practices](#branching-practices)
- [Pull Request Process](#pull-request-process)

## Development Setup

### Prerequisites
- Ubuntu 22.04 (recommended) or compatible Linux distribution
- ROS2 Humble (or your target ROS2 distribution)
- Git
- Python 3.10+
- colcon build tools

### Initial Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/UTRA-ART/timbot.git
   cd timbot
   ```

2. **Install ROS2 dependencies:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop python3-colcon-common-extensions
   ```

3. **Source ROS2:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

4. **Install workspace dependencies:**
   ```bash
   rosdep install --from-paths . --ignore-src -r -y
   ```

## ROS2 Package Structure

This repository is organized into functional directories, each containing multiple ROS2 packages:

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

Each directory can contain one or more ROS2 packages related to its domain.

## Creating ROS2 Packages

### Creating a Python Package

To create a new Python ROS2 package in a specific directory:

```bash
cd <directory>  # e.g., cd cv
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy
```

**Example:** Creating a camera processing package in the `cv` directory:
```bash
cd cv
ros2 pkg create --build-type ament_python camera_processing --dependencies rclpy sensor_msgs cv_bridge
```

### Creating a C++ Package

To create a new C++ ROS2 package:

```bash
cd <directory>  # e.g., cd odom
ros2 pkg create --build-type ament_cmake <package_name> --dependencies rclcpp
```

**Example:** Creating a wheel odometry package in the `odom` directory:
```bash
cd odom
ros2 pkg create --build-type ament_cmake wheel_odom --dependencies rclcpp nav_msgs geometry_msgs
```

### Common Package Dependencies

Here are common dependencies for different types of packages:

- **Computer Vision (cv):** `rclpy`, `sensor_msgs`, `cv_bridge`, `image_transport`
- **Odometry (odom):** `rclcpp`, `nav_msgs`, `geometry_msgs`, `tf2`, `tf2_ros`
- **Navigation (nav):** `rclcpp`, `nav2_msgs`, `geometry_msgs`, `nav_msgs`
- **Sensor Drivers:** `rclcpp`, `sensor_msgs`, `std_msgs`
- **Motor Control:** `rclcpp`, `std_msgs`, `control_msgs`

### Adding Multiple Packages in a Directory

You can have multiple packages in each directory. For example, the `cv` directory might contain:
```
cv/
├── camera_driver/
├── image_processing/
├── object_detection/
└── vision_utils/
```

## Building and Testing

### Building the Workspace

From the root of the repository:

```bash
# Build all packages
colcon build

# Build specific packages
colcon build --packages-select <package_name>

# Build with symlink install (useful for Python packages during development)
colcon build --symlink-install
```

### Sourcing the Workspace

After building, source the workspace:

```bash
source install/setup.bash
```

**Tip:** Add this to your `~/.bashrc` for convenience:
```bash
echo "source /home/czarhc/projects/timbot/install/setup.bash" >> ~/.bashrc
```

### Testing

```bash
# Run all tests
colcon test

# Test specific package
colcon test --packages-select <package_name>

# View test results
colcon test-result --all
```

## Branching Practices

We follow a structured branching strategy to maintain code quality and enable parallel development.

### Branch Types

1. **`main` branch**
   - Protected branch
   - Always contains stable, production-ready code
   - Direct commits are not allowed
   - All changes must come through pull requests

2. **`develop` branch** (if used)
   - Integration branch for features
   - More frequently updated than main
   - Used for testing integrated features

3. **Feature branches**
   - Created from `main` (or `develop`)
   - Naming convention: `feature/<short-description>`
   - Example: `feature/camera-calibration`, `feature/pid-controller`

4. **Bugfix branches**
   - Created from `main` (or `develop`)
   - Naming convention: `bugfix/<short-description>`
   - Example: `bugfix/odom-drift`, `bugfix/sensor-timeout`

5. **Hotfix branches**
   - Created from `main` for urgent fixes
   - Naming convention: `hotfix/<short-description>`
   - Example: `hotfix/motor-safety`

### Branch Naming Conventions

- Use lowercase with hyphens
- Be descriptive but concise
- Include the issue number if applicable: `feature/123-add-lidar-driver`

**Examples:**
```bash
feature/april-tag-detection
feature/kalman-filter-odom
bugfix/imu-calibration
hotfix/emergency-stop
```

### Workflow

1. **Create a new branch:**
   ```bash
   git checkout main
   git pull origin main
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes and commit regularly:**
   ```bash
   git add .
   git commit -m "Add descriptive commit message"
   ```

3. **Keep your branch up to date:**
   ```bash
   git checkout main
   git pull origin main
   git checkout feature/your-feature-name
   git merge main
   # Or use rebase for a cleaner history:
   # git rebase main
   ```

4. **Push your branch:**
   ```bash
   git push origin feature/your-feature-name
   ```

5. **Create a pull request** on GitHub

### Commit Message Guidelines

Write clear, descriptive commit messages:

- Use the imperative mood ("Add feature" not "Added feature")
- First line: brief summary (50 chars or less)
- Blank line, then detailed description if needed
- Reference issue numbers: `Fixes #123` or `Relates to #456`

**Examples:**
```
Add wheel encoder odometry node

- Implement encoder reading from Arduino
- Calculate linear and angular velocities
- Publish to /odom topic

Fixes #42
```

## Pull Request Process

1. **Before creating a PR:**
   - Ensure all tests pass: `colcon test`
   - Build succeeds without warnings: `colcon build`
   - Code follows ROS2 conventions and style guidelines
   - Update documentation if needed

2. **Creating a PR:**
   - Use a clear, descriptive title
   - Fill out the PR template completely
   - Link related issues
   - Request reviews from relevant team members

3. **During review:**
   - Respond to feedback promptly
   - Make requested changes in new commits
   - Engage in constructive discussion

4. **After approval:**
   - Squash commits if requested
   - Ensure branch is up to date with main
   - Maintainer will merge the PR

### PR Checklist

- [ ] Code builds successfully
- [ ] All tests pass
- [ ] New tests added for new features
- [ ] Documentation updated
- [ ] Commit messages are clear
- [ ] No merge conflicts with main
- [ ] Code follows project style guidelines

## Code Style Guidelines

### Python
- Follow [PEP 8](https://pep8.org/)
- Use 4 spaces for indentation
- Maximum line length: 100 characters
- Use meaningful variable and function names

### C++
- Follow [ROS2 C++ Style Guide](https://docs.ros.org/en/humble/Contributing/Code-Style-Language-Versions.html)
- Use 2 spaces for indentation
- Use snake_case for functions and variables
- Use PascalCase for classes

## Getting Help

- Open an issue for bugs or feature requests
- Join our team communication channels
- Consult [ROS2 documentation](https://docs.ros.org/en/humble/)
- Ask questions in pull requests or issues

## License

By contributing to Timbot, you agree that your contributions will be licensed under the project's license.

---

Thank you for contributing to Timbot! 🤖
