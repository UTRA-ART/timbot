#!/bin/bash
# Container entrypoint: build (dev only), display for Gazebo's sensors,
# Foxglove bridge, then the sim. Options (environment variables):
#   TIMBOT_CONFIG=<file>  launch config in launch/config/ (default sim_headless.yaml)
#   TIMBOT_CV=1           simulated ZED camera + lane/depth detection (heavy without a GPU)
#   TIMBOT_GPU=1          render on the host GPU; needs the GPU passed in and the
#                         host's X display (DISPLAY + /tmp/.X11-unix) mounted. Linux only.
#   TIMBOT_BUILD=1        colcon build the mounted source first (set by the dev image)
set -e
source /opt/ros/humble/setup.bash

if [ "${TIMBOT_BUILD:-0}" = 1 ]; then
    if [ ! -f /ws/src/timbot/launch/package.xml ]; then
        echo "Dev mode: mount the timbot repo at /ws/src/timbot (see compose.yaml)." >&2
        exit 1
    fi
    cd /ws
    colcon build --symlink-install --packages-skip zed_open_source \
        --event-handlers console_cohesion- status- summary+
fi
source /ws/install/setup.bash

# Runtime copy of the config with the overrides applied (launch accepts an absolute path).
share=$(ros2 pkg prefix timbot_launch)/share/timbot_launch/config
CONFIG=/tmp/run_config.yaml
cp "$share/${TIMBOT_CONFIG:-sim_headless.yaml}" "$CONFIG"
[ "${TIMBOT_CV:-0}" = 1 ] && sed -i "s/^cv_enabled: false/cv_enabled: true/" "$CONFIG"

if [ "${TIMBOT_GPU:-0}" = 1 ]; then
    ldconfig  # pick up the NVIDIA libraries mounted in by the container runtime
    sed -i "s/software_rendering: '1'/software_rendering: '0'/" "$CONFIG"
else
    # Gazebo's sensors need an OpenGL context even with no GUI.
    # A restarted container keeps the last run's lock; its PID may now be
    # another process, which makes Xvfb refuse to start.
    rm -f /tmp/.X99-lock /tmp/.X11-unix/X99
    Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp &
    export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1
fi

ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765 &
# Foxglove's 3D panel publishes poses to /move_base_simple/goal unless its layout
# says otherwise; Nav2 only listens on /goal_pose.
ros2 run topic_tools relay /move_base_simple/goal /goal_pose &

exec ros2 launch timbot_launch timbot.launch.py config:="$CONFIG" "$@"
