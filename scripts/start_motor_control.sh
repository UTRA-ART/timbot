#!/bin/bash

# Wait until the Pi gets its IP address from the laptop
while ! hostname -I | grep -q "10.42"; do
  echo "Waiting for IP address..."
  sleep 1
done

# Source the base ROS 2 installation
source /opt/ros/humble/setup.bash

# Source your UTRA timbot workspace
source /home/utrapi/timbot/install/setup.bash

# Launch the motor control node
ros2 launch motor_control motor_control.launch.py