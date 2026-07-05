#!/bin/bash
# Source ROS2 + our workspace so `ros2 launch drone_system ...` just works.
set -e
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export PX4_DIR="${PX4_DIR:-/opt/PX4-Autopilot}"
exec "$@"
