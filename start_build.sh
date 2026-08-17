#!/bin/bash
set -eo pipefail

ROBOT_WS="${ROBOT_WS:-/home/harsh/ros2_ws}"
LIDAR_ROOT="$ROBOT_WS/src/ldlidar_stl_ros2"
LIDAR_LOG_SOURCE="$LIDAR_ROOT/ldlidar_driver/src/logger/log_module.cpp"
LIDAR_PATCH="$ROBOT_WS/patches/ldlidar-pthread.patch"

source /opt/ros/jazzy/setup.bash
set -u
cd "$ROBOT_WS"

# Upstream uses pthread functions without including their declarations. Apply
# the pinned compatibility patch once so clean Pi clones build reproducibly.
if ! grep -Fxq '#include <pthread.h>' "$LIDAR_LOG_SOURCE"; then
  git -C "$LIDAR_ROOT" apply "$LIDAR_PATCH"
fi

# Build the robot package and any source dependencies in this workspace.
colcon build --symlink-install --packages-up-to my_robot_package
