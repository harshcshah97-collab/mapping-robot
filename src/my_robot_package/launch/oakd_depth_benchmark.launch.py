"""Publish an OAK-D cloud for benchmarking without starting any motors."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("my_robot_package")
    urdf_path = os.path.join(package_share, "urdf", "my_robot.urdf")
    tracking_config = os.path.join(
        package_share, "config", "person_tracking.yaml"
    )
    hardware_config = os.path.join(
        package_share, "config", "hardware_calibration.yaml"
    )
    with open(urdf_path, "r") as stream:
        robot_description = stream.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="my_robot_package",
            executable="person_follower_node",
            name="person_follower_node",
            output="screen",
            parameters=[
                tracking_config,
                hardware_config,
                {
                    "initial_mode": "idle",
                    "motion_enabled": False,
                    "require_lidar": False,
                    "publish_pointcloud": True,
                    "pointcloud_benchmark_mode": True,
                },
            ],
        ),
    ])
