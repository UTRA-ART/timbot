# Timbot Setup

## 1. Base setup
Complete **Updated_ROS2_WSL_Setup.pdf**, with three fixes:
- In section 6, don't copy the `LIBGL_ALWAYS_SOFTWARE` line from the PDF (it has curly quotes). Use:
  ```bash
  echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc && source ~/.bashrc
  ```
- Before `conda create`, run:
  ```bash
  conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
  conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
  ```
- In the last `pip3 install` command, remove `--user`.

## 2. Timbot setup
Run in your WSL terminal. Timbot uses system Python, so conda stays off.
```bash
# prep
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros python3-pip -y
conda config --set auto_activate_base false
conda deactivate

# clone
cd ~/ros2_ws/src
git clone https://github.com/UTRA-ART/timbot.git
cd timbot
git submodule update --init --recursive
chmod +x nav/load_waypoints/src/nav_autonomous_relay.py

# python packages (order matters; the last two lines undo parts of
# requirements.txt that break ROS, so ignore pip's warnings about them)
/usr/bin/python3 -m pip install --user -r requirements.txt utm
/usr/bin/python3 -m pip install --user "numpy<2"
/usr/bin/python3 -m pip uninstall -y opencv-python opencv-contrib-python setuptools

# build
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-skip zed_open_source

# gazebo model
git clone --depth 1 https://github.com/osrf/gazebo_models.git /tmp/gazebo_models
mkdir -p ~/.gazebo/models
mv /tmp/gazebo_models/construction_barrel ~/.gazebo/models/
rm -rf /tmp/gazebo_models
```

## 3. Launch
In each new terminal:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch timbot_launch timbot.launch.py
```

**Optional, any GPU (dedicated or integrated):** launch like this instead to render RViz on the GPU. It picks an NVIDIA GPU if you have one, otherwise your default GPU. Gazebo stays on the CPU, because WSL's GPU driver is missing OpenGL features Gazebo needs (don't pass `software_rendering:=0`; it crashes Gazebo).
```bash
export LIBGL_ALWAYS_SOFTWARE=0 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
ros2 launch timbot_launch timbot.launch.py
```
CV uses an NVIDIA GPU automatically (CUDA is NVIDIA-only; other GPUs fall back to the CPU): `python3 -c "import torch; print(torch.cuda.is_available())"` prints `True` if it's being used. The sim's default classical lane detection doesn't need it; YOLO mode (`lane_detection_mode: 0` in `launch/config/sim.yaml`) runs on CUDA.

## Troubleshooting
- **Sim hangs or old windows are still open:**
  ```bash
  pkill -f "ign gazebo"; pkill -f -- --ros-args; ros2 daemon stop; rm -rf /dev/shm/fastrtps_*
  ```
- **Gazebo layout is broken or models won't load:**
  ```bash
  rm -rf ~/.ignition/gazebo/6/*.config ~/.ignition/fuel ~/.cache/ignition
  ```
- **Build errors after pulling changes:**
  ```bash
  conda deactivate; cd ~/ros2_ws && rm -rf build install log
  colcon build --symlink-install --packages-skip zed_open_source
  ```
