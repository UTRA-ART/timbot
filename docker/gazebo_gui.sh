#!/bin/bash
# Opens the Gazebo GUI for the sim running in this container, on the Windows
# desktop via WSLg. Needs the container started with docker/compose.wslg.yaml.
# From the repo root:  docker compose exec dev bash src/timbot/docker/gazebo_gui.sh
# (or `sim` instead of `dev`). Closing the window leaves the sim running.
source /opt/ros/humble/setup.bash

for s in /tmp/wslg/wsl/X0 /tmp/wslg/windows/X0; do
    if [ -S "$s" ]; then mkdir -p /tmp/.X11-unix; ln -sf "$s" /tmp/.X11-unix/X0; break; fi
done
if [ ! -S /tmp/.X11-unix/X0 ]; then
    echo "No WSLg display found. Start the sim with:" >&2
    echo "  docker compose -f compose.yaml -f docker/compose.wslg.yaml up dev" >&2
    exit 1
fi

server=$(pgrep -f "ign gazebo sim" | head -1)
if [ -z "$server" ]; then
    echo "Gazebo isn't running yet; wait for the sim to start." >&2
    exit 1
fi
# Same model/world search path as the server, so the robot and barrels load.
export "$(tr '\0' '\n' < /proc/"$server"/environ | grep '^IGN_GAZEBO_RESOURCE_PATH=')"

# CPU rendering: GPU rendering under WSL crashes Gazebo (see sim.yaml).
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1
exec ign gazebo -g --render-engine-gui ogre2
