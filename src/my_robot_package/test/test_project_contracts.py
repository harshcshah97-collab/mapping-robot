"""Static integration contracts for launch, config, maps, and packaging."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PYTHON_PACKAGE = PACKAGE_ROOT / "my_robot_package"


def _console_scripts():
    setup_tree = ast.parse((PACKAGE_ROOT / "setup.py").read_text())
    setup_call = next(
        node for node in ast.walk(setup_tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", None) == "setup"
    )
    entry_points = next(
        keyword.value for keyword in setup_call.keywords
        if keyword.arg == "entry_points"
    )
    values = ast.literal_eval(entry_points)
    return {
        item.split("=", 1)[0].strip()
        for item in values["console_scripts"]
    }


def _first_party_launch_executables(path):
    tree = ast.parse(path.read_text())
    executables = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", None) != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        package = keywords.get("package")
        executable = keywords.get("executable")
        if not isinstance(package, ast.Constant) or not isinstance(
            executable, ast.Constant
        ):
            continue
        if package.value == "my_robot_package":
            executables.add(executable.value)
    return executables


def test_all_first_party_python_files_parse():
    for path in sorted(PACKAGE_ROOT.rglob("*.py")):
        ast.parse(path.read_text(), filename=str(path))


def test_launch_nodes_have_installed_entry_points():
    installed = _console_scripts()
    launched = set()
    for path in (PACKAGE_ROOT / "launch").glob("*.launch.py"):
        launched.update(_first_party_launch_executables(path))
    assert launched <= installed


def test_every_map_yaml_has_an_image():
    for path in sorted((PACKAGE_ROOT / "maps").glob("*.yaml")):
        metadata = yaml.safe_load(path.read_text())
        image = path.parent / metadata["image"]
        assert image.is_file(), f"{path.name} references missing {image.name}"
        assert float(metadata["resolution"]) > 0.0
        assert 0.0 <= float(metadata["free_thresh"]) < float(
            metadata["occupied_thresh"]
        ) <= 1.0


def test_navigation_sensor_topics_and_behavior_server_are_current():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "nav2_params.yaml").read_text()
    )
    assert "behavior_server" in config
    assert "recoveries_server" not in config

    for costmap_name in ("local_costmap", "global_costmap"):
        parameters = config[costmap_name][costmap_name]["ros__parameters"]
        topics = parameters["range_sensor_layer"]["topics"]
        assert all(topic.startswith("/") for topic in topics)


def test_required_sensor_frames_exist_in_urdf():
    urdf = ET.parse(PACKAGE_ROOT / "urdf" / "my_robot.urdf")
    links = {link.attrib["name"] for link in urdf.findall("link")}
    assert {
        "base_footprint",
        "base_link",
        "base_laser",
        "imu_link",
        "ultrasonic_link",
        "ir_left_link",
        "ir_right_link",
        "oakd_rgb_camera_optical_frame",
    } <= links


def test_each_operating_mode_owns_one_motor_driver():
    for name in (
        "bringup_and_map.launch.py",
        "navigation.launch.py",
        "person_tracking.launch.py",
    ):
        path = PACKAGE_ROOT / "launch" / name
        executables = _first_party_launch_executables(path)
        assert "motor_driver_node" in executables


def test_launch_files_do_not_use_source_tree_absolute_paths():
    for path in (PACKAGE_ROOT / "launch").glob("*.launch.py"):
        assert "/home/harsh/ros2_ws/src" not in path.read_text()


def test_bumper_has_a_direct_motor_stop_path():
    bumper_source = (PYTHON_PACKAGE / "bumper_node.py").read_text()
    motor_source = (PYTHON_PACKAGE / "motor_driver_node.py").read_text()
    assert "'/safety/stop'" in bumper_source
    assert "'/safety/stop'" in motor_source


def test_full_launches_route_motion_through_twist_mux():
    for name in (
        "bringup_and_map.launch.py",
        "navigation.launch.py",
        "person_tracking.launch.py",
    ):
        source = (PACKAGE_ROOT / "launch" / name).read_text()
        assert "package='twist_mux'" in source or 'package="twist_mux"' in source
        assert "/cmd_vel_out" in source

    motor_source = (PYTHON_PACKAGE / "motor_driver_node.py").read_text()
    assert "command_topic" in motor_source


def test_oakd_depth_is_calibration_gated():
    hardware = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "hardware_calibration.yaml").read_text()
    )
    assert not hardware["person_follower_node"]["ros__parameters"][
        "oakd_extrinsics_calibrated"
    ]
    source = (PYTHON_PACKAGE / "person_follower_node.py").read_text()
    assert "self.oakd_extrinsics_calibrated" in source
    assert "self.oakd_depth_benchmark_passed" in source


def test_assistant_service_has_resilient_audio_configuration():
    service = (PACKAGE_ROOT / "web_ui" / "robot_assistant.service").read_text()
    assert "XDG_RUNTIME_DIR=/run/user/%U" in service
    assert "PULSE_SERVER=unix:/run/user/%U/pulse/native" in service
    assert "EnvironmentFile=/home/harsh/.config/mapping-robot/assistant.env" in service

    source = (PYTHON_PACKAGE / "assistant_node.py").read_text()
    assert '"microphone_sample_rate"' in source
    assert "Retrying in 5 seconds" in source
