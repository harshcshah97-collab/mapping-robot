"""Bring up the production DepthAI 2.32 person-following stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("my_robot_package")
    urdf_file = os.path.join(package_share, "urdf", "my_robot.urdf")
    lidar_launch_file = os.path.join(package_share, "launch", "ld19.launch.py")
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
    depthai2_python = LaunchConfiguration("depthai2_python")
    allow_hot_operation = LaunchConfiguration("allow_hot_operation")

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_config],
        remappings=[("/cmd_vel_out", "/cmd_vel_out")],
    )
    motor = Node(
        package="my_robot_package",
        executable="motor_driver_node",
        name="motor_driver_node",
        output="screen",
        parameters=[hardware_config],
    )
    encoder = Node(
        package="my_robot_package",
        executable="encoder_odom_node",
        name="encoder_odom_node",
        output="screen",
        parameters=[hardware_config, {
            "base_frame": "base_footprint",
            "odom_frame": "odom",
            "publish_tf": False,
        }],
    )
    ultrasonic = Node(
        package="my_robot_package",
        executable="ultra_sensor_node",
        name="ultra_sensor_node",
        output="screen",
    )
    infrared = Node(
        package="my_robot_package",
        executable="ir_driver",
        name="ir_sensor_node",
        output="screen",
    )
    bumper = Node(
        package="my_robot_package",
        executable="bumper_node",
        name="bumper_node",
        output="screen",
    )
    follower = ExecuteProcess(
        cmd=[
            depthai2_python,
            "-m",
            "my_robot_package.person_follower_node",
            "--ros-args",
            "--params-file",
            tracking_config,
            "--params-file",
            hardware_config,
            "-p",
            ["initial_mode:=", initial_mode],
            "-p",
            ["require_lidar:=", enable_lidar],
            "-p",
            ["allow_hot_operation:=", allow_hot_operation],
            "-r",
            "/cmd_vel:=/cmd_vel/tracking",
        ],
        additional_env={"PYTHONNOUSERSITE": "1"},
        output="screen",
    )

    shutdown_handlers = []
    for action, label in (
        (follower, "DepthAI 2 follower"),
        (twist_mux, "velocity multiplexer"),
        (motor, "motor driver"),
        (encoder, "encoder odometry"),
        (ultrasonic, "ultrasonic sensor"),
        (infrared, "IR sensors"),
        (bumper, "bumper"),
    ):
        shutdown_handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=action,
                    on_exit=[EmitEvent(event=Shutdown(reason="%s exited" % label))],
                )
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "initial_mode",
            default_value="idle",
            description="Safe default is idle. Supported: idle, follow, keep_frame.",
        ),
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="true",
            description="Start and require the LD19 obstacle-stop sensor.",
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
        DeclareLaunchArgument(
            "depthai2_python",
            default_value="/home/harsh/ros2_ws/.venv-depthai2/bin/python3",
            description="Interpreter containing exactly DepthAI 2.32.0.0.",
        ),
        DeclareLaunchArgument(
            "allow_hot_operation",
            default_value="false",
            description=(
                "Explicitly bypass Pi heat/throttle stops for a supervised run; "
                "undervoltage and the absolute temperature stop remain active."
            ),
        ),
        LogInfo(
            msg=(
                "Starting production DepthAI 2 tracking in IDLE. Motion requires "
                "a follow command plus fresh LiDAR, ultrasonic, IR, and bumper data."
            )
        ),
        *shutdown_handlers,
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
        twist_mux,
        motor,
        encoder,
        ultrasonic,
        infrared,
        bumper,
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
        follower,
    ])
