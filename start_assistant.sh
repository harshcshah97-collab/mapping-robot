#!/bin/bash

# Source ROS 2 and your workspace
source /opt/ros/jazzy/setup.bash
source /home/harsh/ros2_ws/install/setup.bash

# Export your API Key (Paste your actual key from your .bashrc here!)
export OPENAI_API_KEY="${OPENAI_API_KEY:?Set OPENAI_API_KEY before running}"

# Run the assistant node
ros2 run my_robot_package assistant_node