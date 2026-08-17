#!/bin/bash
set -eo pipefail

ROBOT_WS="${ROBOT_WS:-/home/harsh/ros2_ws}"

source /opt/ros/jazzy/setup.bash
set -u
cd "$ROBOT_WS"

# Build the robot package and any source dependencies in this workspace.
colcon build --symlink-install --packages-up-to my_robot_package
