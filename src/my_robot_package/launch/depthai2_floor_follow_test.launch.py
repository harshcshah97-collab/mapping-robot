"""Minimal, isolated launch for one bounded DepthAI 2 floor-follow test."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("my_robot_package")
    hardware_config = os.path.join(
        package_share, "config", "hardware_calibration.yaml"
    )
    test_config = os.path.join(
        package_share, "config", "depthai2_floor_follow_test.yaml"
    )

    enable_motion = LaunchConfiguration("enable_motion")
    allow_hot_test = LaunchConfiguration("allow_hot_test")
    maximum_wait_sec = LaunchConfiguration("maximum_wait_sec")
    depthai2_python = LaunchConfiguration("depthai2_python")
    maximum_wall_time = LaunchConfiguration("maximum_wall_time")

    lidar = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="LD19",
        output="screen",
        condition=IfCondition(enable_motion),
        parameters=[{
            "product_name": "LDLiDAR_LD19",
            "topic_name": "scan",
            "frame_id": "base_laser",
            "port_name": "/dev/ttyUSB0",
            "port_baudrate": 230400,
            "laser_scan_dir": True,
            "enable_angle_crop_func": False,
            "angle_crop_min": 135.0,
            "angle_crop_max": 225.0,
        }],
    )
    bumper = Node(
        package="my_robot_package",
        executable="bumper_node",
        name="bumper_node",
        output="screen",
        condition=IfCondition(enable_motion),
    )
    encoder = Node(
        package="my_robot_package",
        executable="encoder_odom_node",
        name="encoder_odom_node",
        output="screen",
        condition=IfCondition(enable_motion),
        parameters=[hardware_config, {
            "publish_tf": False,
            "force_zero_translation_while_turning": False,
        }],
    )
    motor = Node(
        package="my_robot_package",
        executable="motor_driver_node",
        name="motor_driver_node",
        output="screen",
        condition=IfCondition(enable_motion),
        parameters=[hardware_config, {
            "command_topic": "/depthai2_test/cmd_vel",
            "command_timeout_sec": 0.25,
            "closed_loop_enabled": False,
        }],
    )
    controller = ExecuteProcess(
        cmd=[
            depthai2_python,
            "-m",
            "my_robot_package.depthai2_floor_follow_test_node",
            "--ros-args",
            "--params-file",
            test_config,
            "-p",
            ["motion_enabled:=", enable_motion],
            "-p",
            ["allow_hot_test:=", allow_hot_test],
            "-p",
            ["maximum_wait_sec:=", maximum_wait_sec],
        ],
        additional_env={"PYTHONNOUSERSITE": "1"},
        output="screen",
    )

    shutdown_handlers = []
    for action, label in (
        (controller, "DepthAI2 controller"),
        (lidar, "LiDAR"),
        (bumper, "bumper"),
        (encoder, "encoder odometry"),
        (motor, "motor driver"),
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
            "enable_motion",
            default_value="false",
            description="Must be explicitly true before the motor node is started.",
        ),
        DeclareLaunchArgument(
            "allow_hot_test",
            default_value="false",
            description=(
                "One-shot override for Pi heat and heat-throttling gates. "
                "Undervoltage and the 90 C absolute stop remain active."
            ),
        ),
        DeclareLaunchArgument(
            "depthai2_python",
            default_value="/home/harsh/ros2_ws/.venv-depthai2/bin/python3",
            description="Python interpreter containing exactly depthai 2.32.0.0.",
        ),
        DeclareLaunchArgument(
            "maximum_wait_sec",
            default_value="35.0",
            description="Maximum disarmed wait for a valid target and arm request.",
        ),
        DeclareLaunchArgument(
            "maximum_wall_time",
            default_value="40.0",
            description="Independent whole-launch timeout, including preflight.",
        ),
        LogInfo(msg="Starting isolated DepthAI2 test DISARMED."),
        *shutdown_handlers,
        lidar,
        bumper,
        encoder,
        motor,
        controller,
        TimerAction(
            period=maximum_wall_time,
            actions=[EmitEvent(event=Shutdown(reason="bounded test wall timeout"))],
        ),
    ])
