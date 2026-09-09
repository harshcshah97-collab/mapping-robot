import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PYTHON_PACKAGE = PACKAGE_ROOT / "my_robot_package"


def test_test_mode_defaults_to_disarmed_and_is_tightly_bounded():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "depthai2_floor_follow_test.yaml").read_text()
    )["depthai2_floor_follow_test"]["ros__parameters"]
    assert config["motion_enabled"] is False
    assert config["allow_hot_test"] is False
    assert config["session_duration_sec"] == 30.0
    assert config["target_distance_m"] == 0.60
    assert config["distance_deadband_m"] == 0.0
    assert config["maximum_linear_mps"] <= 0.06
    assert config["maximum_angular_rps"] <= 0.20
    assert config["lidar_stop_distance_m"] >= 0.60
    assert config["maximum_start_pi_temp_c"] < config["maximum_run_pi_temp_c"]


def test_launch_is_private_minimal_and_shuts_down_on_required_exit():
    source = (
        PACKAGE_ROOT / "launch" / "depthai2_floor_follow_test.launch.py"
    ).read_text()
    ast.parse(source)
    assert 'default_value="false"' in source
    assert '"/depthai2_test/cmd_vel"' in source
    assert '"command_timeout_sec": 0.25' in source
    assert "OnProcessExit" in source
    assert "TimerAction" in source
    assert "person_follower_node" not in source
    assert "twist_mux" not in source
    assert "assistant_node" not in source
    assert '"-m"' in source
    assert '"PYTHONNOUSERSITE": "1"' in source


def test_node_has_version_sensor_and_thermal_gates():
    source = (
        PYTHON_PACKAGE / "depthai2_floor_follow_test_node.py"
    ).read_text()
    ast.parse(source)
    for contract in (
        'REQUIRED_DEPTHAI_VERSION = "2.32.0.0"',
        'PERSON_LABEL = 15',
        '"lidar_stale"',
        '"bumper_heartbeat_stale"',
        '"pi_over_temperature"',
        '"pi_absolute_over_temperature"',
        '"completed_bounded_test"',
        'self._publish_zero()',
    ):
        assert contract in source
