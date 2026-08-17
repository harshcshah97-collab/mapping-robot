"""Bring up the robot base and OAK-D tracking without loading a saved map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("my_robot_package")
    urdf_file = os.path.join(package_share, "urdf", "my_robot.urdf")
    lidar_launch_file = os.path.join(package_share, "launch", "ld19.launch.py")
    ekf_config = os.path.join(package_share, "config", "ekf.yaml")
    tracking_config = os.path.join(package_share, "config", "person_tracking.yaml")
    hardware_config = os.path.join(
        package_share, "config", "hardware_calibration.yaml"
    )
    twist_mux_config = os.path.join(package_share, "config", "twist_mux.yaml")

    with open(urdf_file, "r") as urdf_stream:
        robot_description = urdf_stream.read()

    initial_mode = LaunchConfiguration("initial_mode")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_assistant = LaunchConfiguration("enable_assistant")

    return LaunchDescription([
        DeclareLaunchArgument(
            "initial_mode",
            default_value="idle",
            description="Safe default is idle. Supported: idle, follow, keep_frame.",
        ),
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="true",
            description="Start LD19 for obstacle-stop protection.",
        ),
        DeclareLaunchArgument(
            "enable_assistant",
            default_value="false",
            description=(
                "Start Bob inside this launch. Keep false when the recommended "
                "robot_assistant.service already runs Bob at boot."
            ),
        ),
        DeclareLaunchArgument(
            "enable_foxglove",
            default_value="false",
            description="Start Foxglove for camera and tracking diagnostics.",
        ),
        LogInfo(
            msg="Starting standalone tracking mode (no saved map or Nav2 required)."
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_file),
            condition=IfCondition(enable_lidar),
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[twist_mux_config],
            remappings=[("/cmd_vel_out", "/cmd_vel_out")],
        ),
        Node(
            package="my_robot_package",
            executable="motor_driver_node",
            name="motor_driver_node",
            output="screen",
            parameters=[hardware_config],
        ),
        Node(
            package="my_robot_package",
            executable="encoder_odom_node",
            name="encoder_odom_node",
            output="screen",
            parameters=[hardware_config, {
                "base_frame": "base_footprint",
                "odom_frame": "odom",
                "publish_tf": False,
            }],
        ),
        Node(
            package="my_robot_package",
            executable="imu_node",
            name="imu_node",
            output="screen",
            parameters=[hardware_config],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config],
        ),
        Node(
            package="my_robot_package",
            executable="ultra_sensor_node",
            name="ultra_sensor_node",
            output="screen",
        ),
        Node(
            package="my_robot_package",
            executable="ir_driver",
            name="ir_sensor_node",
            output="screen",
        ),
        Node(
            package="my_robot_package",
            executable="bumper_node",
            name="bumper_node",
            output="screen",
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{"port": 8765}],
            condition=IfCondition(enable_foxglove),
        ),
        Node(
            package="my_robot_package",
            executable="assistant_node",
            name="assistant_node",
            output="screen",
            parameters=[{"allow_shell_commands": False}],
            condition=IfCondition(enable_assistant),
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
                    "initial_mode": initial_mode,
                    "require_lidar": ParameterValue(enable_lidar, value_type=bool),
                },
            ],
            remappings=[("/cmd_vel", "/cmd_vel/tracking")],
        ),
    ])
