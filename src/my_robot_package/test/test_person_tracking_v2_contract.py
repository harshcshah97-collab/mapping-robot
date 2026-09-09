"""Static contracts for the production DepthAI 2 person-tracking stack."""

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PYTHON_PACKAGE = PACKAGE_ROOT / "my_robot_package"


def test_apartment_tracking_configuration_uses_every_near_field_sensor():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "person_tracking.yaml").read_text()
    )["/**"]["ros__parameters"]
    assert config["target_distance_m"] == 0.60
    assert config["distance_deadband_m"] == 0.0
    assert config["follow_speed"] == 0.10
    assert config["lidar_stop_distance_m"] == 0.30
    assert config["range_stop_distance_m"] == 0.20
    assert config["ir_stop_requires_both"] is True
    assert config["require_lidar"] is True
    assert config["require_ultrasonic"] is True
    assert config["require_ir"] is True
    assert config["require_bumper"] is True
    assert config["allow_hot_operation"] is False
    assert config["absolute_stop_pi_temp_c"] == 95.0
    assert config["require_enrollment_for_motion"] is True


def test_normal_launch_uses_isolated_depthai2_and_starts_idle():
    source = (PACKAGE_ROOT / "launch" / "person_tracking.launch.py").read_text()
    ast.parse(source)
    assert 'default_value="idle"' in source
    assert '"my_robot_package.person_follower_node"' in source
    assert '"PYTHONNOUSERSITE": "1"' in source
    assert '"/cmd_vel:=/cmd_vel/tracking"' in source
    assert "OnProcessExit" in source
    for executable in (
        "motor_driver_node",
        "encoder_odom_node",
        "ultra_sensor_node",
        "ir_driver",
        "bumper_node",
    ):
        assert 'executable="%s"' % executable in source


def test_normal_follower_is_v2_spatial_and_fail_closed_on_sensor_loss():
    source = (PYTHON_PACKAGE / "person_follower_node.py").read_text()
    ast.parse(source)
    for contract in (
        'REQUIRED_DEPTHAI_VERSION = "2.32.0.0"',
        "PERSON_LABEL = 15",
        "MobileNetSpatialDetectionNetwork",
        'required_ranges.append("ultrasonic")',
        'required_ranges.extend(("ir_left", "ir_right"))',
        '"waiting_for_%s" % name',
        'if self.ir_stop_requires_both:',
        'if len(ir_obstacles) == 2:',
        '"motion_blocked:target_not_enrolled"',
        '"enrollment_requires_exactly_one_person"',
        '"enrolled_subject_cannot_be_verified"',
        '"waiting_for_bumper"',
        '"lidar_obstacle"',
        '"pi_undervoltage"',
        '"pi_absolute_over_temperature"',
    ):
        assert contract in source
