# Building timbot

## Prerequisites

### 1. Install ROS2 Humble
```bash
# Follow official ROS2 Humble installation guide
# https://docs.ros.org/en/humble/Installation.html
```

### 2. Setup Python Environment

#### Option A: Using Conda (Recommended for Multi-Project Setups)

If you work on multiple ROS2 projects with different Python dependencies:

```bash
# Create a Conda environment for this project
conda create -n timbot_env python=3.10
conda activate timbot_env

# Install dependencies
pip install -r requirements.txt
```

**Note:** The workspace is configured to automatically detect and use the active Conda environment. No additional CMake configuration needed!

#### Option B: Using System Python

```bash
# Install dependencies globally (not recommended if you have other projects)
pip install -r requirements.txt
```

### 3. Install Python Dependencies
```bash
pip install -r requirements.txt
```

**Important:** Always activate your environment before building:
```bash
conda activate timbot_env  # Or your environment name
```

### 3. Install System Dependencies
```bash
# Install ROS2 packages
sudo apt update
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-tf2-tools \
    ros-humble-cv-bridge

# Install development tools
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep
```

## Building the Workspace

### Clean Build (Recommended for first time)
```bash
cd /home/czarhc/projects/timbot
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Quick Rebuild (After code changes)
```bash
colcon build --symlink-install
```

### Build Specific Package Only
```bash
colcon build --symlink-install --packages-select odom_state
```

### Build with Verbose Output (for debugging)
```bash
colcon build --symlink-install --event-handlers console_direct+
```

## After Building

### Source the Workspace
**You must do this in every new terminal:**
```bash
source install/setup.bash
```

Or add to your `~/.bashrc`:
```bash
echo "source ~/projects/timbot/install/setup.bash" >> ~/.bashrc
```

### Verify Installation
```bash
# Check packages are found
ros2 pkg list | grep -E "odom_state|lane_detection|load_waypoints|nav_stack"

# Check executables
ros2 pkg executables odom_state
ros2 pkg executables lane_detection
```

## Common Build Issues

### Issue: "No module named 'em'"
```bash
pip install empy==3.3.4
```

### Issue: "AttributeError: module 'em' has no attribute 'BUFFERED_OPT'"
```bash
pip uninstall empy em
pip install empy==3.3.4
```

### Issue: Using Conda and getting Python errors
```bash
# Make sure Conda environment is active
conda activate timbot_env

# Reinstall dependencies
pip install -r requirements.txt

# Rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Issue: CMake warnings about Python library conflicts
This is normal when using Conda environments and can be safely ignored if your packages work correctly. The workspace is configured to handle this automatically.

### Issue: CMake can't find packages
```bash
# Install missing ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## Testing

See individual package READMEs for testing instructions:
- `odom/README.md` - Odometry and localization
- `cv/README.md` - Computer vision and lane detection
- `nav/README.md` - Navigation and path planning
