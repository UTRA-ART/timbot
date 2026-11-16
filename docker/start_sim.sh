#!/bin/bash
# Container entrypoint: build (dev only), display for Gazebo's sensors,
# Foxglove bridge, then the sim. Options (environment variables):
#   TIMBOT_CONFIG=<file>  launch config in launch/config/ (default sim_headless.yaml)
#   TIMBOT_CV=1           simulated ZED camera + lane/depth detection (heavy without a GPU)
#   TIMBOT_LOG_LEVEL=<l>  set every stage's log_level (debug|info|warn|error), e.g. to
#                         see node logs in Foxglove's Log panel (/rosout)
#   TIMBOT_WORLD=0        don't publish the Gazebo world's scenery (/gazebo_world*)
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
if [ -n "${TIMBOT_LOG_LEVEL:-}" ]; then
    case "$TIMBOT_LOG_LEVEL" in
        debug|info|warn|error) ;;
        *) echo "TIMBOT_LOG_LEVEL must be debug, info, warn or error (got '$TIMBOT_LOG_LEVEL')." >&2; exit 1 ;;
    esac
    sed -i -E "s/^(\s+log_level:\s*)[a-z]+/\1$TIMBOT_LOG_LEVEL/" "$CONFIG"
fi

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

# use_sim_time: Foxglove then stamps the goals it publishes with sim time, not the
# wall clock (Nav2 can't plan for a goal stamped decades in the sim's future).
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765 use_sim_time:=true &
# Foxglove's 3D panel publishes poses to /move_base_simple/goal unless its layout
# says otherwise; Nav2 only listens on /goal_pose.
ros2 run topic_tools relay /move_base_simple/goal /goal_pose &

# The Gazebo world's scenery as markers on /gazebo_world (Foxglove can't show Gazebo itself).
world_args=$(python3 - "$CONFIG" "${TIMBOT_WORLD:-1}" <<'EOF'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1]))
g, s = c.get('gazebo') or {}, c.get('spawn') or {}
if g.get('enabled') and s.get('enabled') and sys.argv[2] != '0':
    print(f"-p world_file:={g.get('world_file', 'track.world')} -p spawn_x:={s.get('x', 0)}"
          f" -p spawn_y:={s.get('y', 0)} -p spawn_yaw:={s.get('yaw', 0)}"
          " -p true_pose_topic:=/gazebo/_true_poses")
EOF
)
if [ -n "$world_args" ]; then
    ros2 run gazebo_worlds world_markers.py --ros-args $world_args &
    # Gazebo's true model poses (all our worlds are named "default"), for the ground-truth ghost.
    # "_" hides the topic from Foxglove, which would otherwise subscribe to every TF-type topic.
    ros2 run ros_gz_bridge parameter_bridge \
        '/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V' \
        --ros-args -p lazy:=true -r /world/default/dynamic_pose/info:=/gazebo/_true_poses &
fi

exec ros2 launch timbot_launch timbot.launch.py config:="$CONFIG" "$@"
