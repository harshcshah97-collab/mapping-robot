#!/bin/bash
set -euo pipefail

ROBOT_WS="${ROBOT_WS:-/home/harsh/ros2_ws}"

# Source ROS 2 and the robot workspace.
source /opt/ros/jazzy/setup.bash
source "$ROBOT_WS/install/setup.bash"

# Keep the key in the service/user environment; never put it in this repository.
export OPENAI_API_KEY="${OPENAI_API_KEY:?Set OPENAI_API_KEY before running}"

exec ros2 run my_robot_package assistant_node --ros-args \
  -p allow_shell_commands:=false "$@"
