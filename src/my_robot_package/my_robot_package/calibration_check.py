#!/usr/bin/env python3
"""Validate calibration gates and shared robot geometry configuration."""

import argparse
from pathlib import Path
import sys

import yaml


REQUIRED_WHEEL_FLAGS = (
    "wheel_geometry_measured",
    "wheel_speed_measured",
    "closed_loop_tuned_wheels_raised",
    "closed_loop_tuned_on_floor",
)


def default_config_directory():
    """Locate package config from an install or a source checkout."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("my_robot_package")) / "config"
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[1] / "config"


def calibration_findings(config_directory):
    """Return ``(errors, incomplete)`` for the calibration configuration."""
    hardware = yaml.safe_load(
        (config_directory / "hardware_calibration.yaml").read_text()
    )
    status = yaml.safe_load(
        (config_directory / "calibration_status.yaml").read_text()
    )

    motor = hardware["motor_driver_node"]["ros__parameters"]
    encoder = hardware["encoder_odom_node"]["ros__parameters"]
    camera = hardware["person_follower_node"]["ros__parameters"]
    perception_camera = hardware["oakd_perception_node"]["ros__parameters"]
    errors = []
    incomplete = [name for name, complete in status.items() if not complete]

    separation_error = abs(
        float(motor["wheel_separation_m"])
        - float(encoder["wheel_separation"])
    )
    if separation_error > 1e-6:
        errors.append(
            "Motor and encoder wheel-separation values do not match."
        )

    for key in (
        "oakd_extrinsics_calibrated",
        "oakd_depth_benchmark_passed",
    ):
        if camera[key] != perception_camera[key]:
            errors.append(
                "%s must match for tracking and perception node names." % key
            )

    wheels_complete = all(status.get(name, False) for name in REQUIRED_WHEEL_FLAGS)
    if motor["wheel_calibration_confirmed"] and not wheels_complete:
        errors.append(
            "wheel_calibration_confirmed is true before all wheel checks passed."
        )
    if motor["closed_loop_enabled"] and not motor["wheel_calibration_confirmed"]:
        errors.append(
            "closed_loop_enabled requires wheel_calibration_confirmed."
        )
    if (
        camera["oakd_extrinsics_calibrated"]
        and not status.get("oakd_mount_transform_measured", False)
    ):
        errors.append(
            "OAK-D extrinsics are enabled before the mount transform was measured."
        )
    if (
        camera["oakd_depth_benchmark_passed"]
        and not status.get("oakd_depth_benchmark_passed", False)
    ):
        errors.append(
            "OAK-D depth is enabled before the benchmark was recorded as passed."
        )
    return errors, incomplete


def main(arguments=None):
    parser = argparse.ArgumentParser(
        description="Check companion-robot physical calibration gates."
    )
    parser.add_argument(
        "--config-dir",
        type=Path,
        default=default_config_directory(),
        help="Directory containing hardware_calibration.yaml.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Also fail when any physical checklist item is incomplete.",
    )
    args = parser.parse_args(arguments)

    try:
        errors, incomplete = calibration_findings(args.config_dir)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        print("CALIBRATION CONFIG ERROR: %s" % error, file=sys.stderr)
        return 2

    if errors:
        for error in errors:
            print("ERROR: %s" % error)
    else:
        print("Calibration gates are internally consistent.")

    if incomplete:
        print("Physical checks still required:")
        for item in incomplete:
            print("  - %s" % item)
    else:
        print("All recorded physical calibration checks are complete.")

    if errors or (args.strict and incomplete):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
