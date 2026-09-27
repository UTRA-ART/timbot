#!/bin/bash
# Run once via `pixi run first-time-setup`. Mirrors setup.md §2 / barrel_setup.txt's
# manual steps, adapted for the pixi/RoboStack environment:
#   - initializes submodules
#   - marks the hardware-only vendor drivers (real USB/serial sensors: IMU,
#     LiDAR, GPS, ZED camera) COLCON_IGNORE, the same way CLAUDE.md documents
#     the existing dev machine already does for its local build — none of
#     these are needed for sim.yaml, and building them would require
#     phidgets/rplidar/ZED-SDK system libs this pixi env doesn't provide.
#     misc/twist_mux is NOT ignored: it's needed in both sim and real modes
#     (robot_bringup.launch.py) and is UTRA's own forked config, so it's
#     built from source like any other first-party package.
#   - fixes nav_autonomous_relay.py's exec bit
#   - fetches the Gazebo construction_barrel model
set -e

cd "$(dirname "$0")/.."

if git submodule status | grep -q '^-'; then
    echo "[first-time-setup] Initializing submodules..."
    git submodule update --init --recursive
fi

for pkg in sensor_drivers/imu sensor_drivers/navsat sensor_drivers/rplidar sensor_drivers/zed_open_source misc/ds4_driver; do
    if [ -d "$pkg" ] && [ ! -f "$pkg/COLCON_IGNORE" ]; then
        touch "$pkg/COLCON_IGNORE"
        echo "[first-time-setup] COLCON_IGNORE: $pkg (hardware-only, not needed for sim)"
    fi
done

chmod +x nav/load_waypoints/src/nav_autonomous_relay.py

if [ ! -d "$HOME/.gazebo/models/construction_barrel" ]; then
    echo "[first-time-setup] Fetching Gazebo construction_barrel model..."
    tmp_dir="$(mktemp -d)"
    git clone --depth 1 https://github.com/osrf/gazebo_models.git "$tmp_dir"
    mkdir -p "$HOME/.gazebo/models"
    mv "$tmp_dir/construction_barrel" "$HOME/.gazebo/models/"
    rm -rf "$tmp_dir"
fi

echo "[first-time-setup] Done. Next: pixi run build"
