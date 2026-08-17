#!/bin/bash
set -eo pipefail

ROBOT_WS="${ROBOT_WS:-/home/harsh/ros2_ws}"

# Source ROS 2 and the robot workspace.
source /opt/ros/jazzy/setup.bash
source "$ROBOT_WS/install/setup.bash"
set -u

# System services do not inherit the logged-in user's PipeWire environment.
# Resolve it at runtime from the actual service user instead of a systemd
# specifier, which expands to root in a system-level unit.
ROBOT_USER_ID="$(id -u)"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$ROBOT_USER_ID}"
export PULSE_SERVER="${PULSE_SERVER:-unix:$XDG_RUNTIME_DIR/pulse/native}"

# Keep the key in the service/user environment; never put it in this repository.
export OPENAI_API_KEY="${OPENAI_API_KEY:?Set OPENAI_API_KEY before running}"

exec ros2 run my_robot_package assistant_node --ros-args \
  -p allow_shell_commands:=false "$@"
