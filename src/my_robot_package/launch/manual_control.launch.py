"""Bring up the minimal, bumper-protected manual driving stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("my_robot_package")
    hardware_config = os.path.join(
        package_share, "config", "hardware_calibration.yaml"
    )
    twist_mux_config = os.path.join(
        package_share, "config", "twist_mux.yaml"
    )

    multiplexer = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_config],
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
        parameters=[hardware_config, {"publish_tf": False}],
    )
    bumper = Node(
        package="my_robot_package",
        executable="bumper_node",
        name="bumper_node",
        output="screen",
    )

    shutdown_handlers = []
    for action, label in (
        (multiplexer, "velocity multiplexer"),
        (motor, "motor driver"),
        (encoder, "encoder odometry"),
        (bumper, "bumper"),
    ):
        shutdown_handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=action,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="%s exited" % label))
                    ],
                )
            )
        )

    return LaunchDescription(
        [*shutdown_handlers, multiplexer, motor, encoder, bumper]
    )
