#!/bin/bash
set -eo pipefail

ROBOT_WS="${ROBOT_WS:-/home/harsh/ros2_ws}"

source /opt/ros/jazzy/setup.bash
source "$ROBOT_WS/install/setup.bash"
set -u

exec ros2 launch my_robot_package bringup_and_map.launch.py "$@"
