"""Static contracts for the local Bob companion app."""

import ast
import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WEB_ROOT = PACKAGE_ROOT / "web_ui"


def test_backend_exposes_mode_room_navigation_and_identity_routes():
    source = (WEB_ROOT / "robot_web_server.py").read_text()
    ast.parse(source)
    for route in (
        "/api/start_follow",
        "/api/start_mapping",
        "/api/start_navigation",
        "/api/navigation_goal",
        "/api/navigate_room",
        "/api/rooms",
        "/api/start_teleop",
        "/api/tracking_command",
        "/api/stop",
    ):
        assert route in source
    assert "Enroll yourself in Vision mode" in source
    assert "start_new_session=True" in source


def test_mobile_app_contains_required_controls_and_room_editor():
    html = (WEB_ROOT / "index.html").read_text()
    script = (WEB_ROOT / "main.js").read_text()
    for control in (
        'id="emergency-stop"',
        'id="start-navigation"',
        'id="start-mapping"',
        'id="draw-room"',
        'id="save-room"',
        'data-tracking="enroll_target"',
        'data-action="start_follow"',
        'id="assistant-form"',
    ):
        assert control in html
    for behavior in (
        "/api/navigation_goal",
        "/api/navigate_room",
        "/assistant/text_query",
        "visibilitychange",
        "stopMoving",
    ):
        assert behavior in script


def test_pwa_manifest_and_manual_launch_contract():
    manifest = json.loads((WEB_ROOT / "manifest.webmanifest").read_text())
    assert manifest["display"] == "standalone"
    assert manifest["start_url"] == "/"
    launch_source = (
        PACKAGE_ROOT / "launch" / "manual_control.launch.py"
    ).read_text()
    ast.parse(launch_source)
    for node in ("twist_mux", "motor_driver_node", "encoder_odom_node", "bumper_node"):
        assert node in launch_source
    assert "OnProcessExit" in launch_source


def test_assistant_routes_modes_and_named_rooms_through_app_manager():
    source = (PACKAGE_ROOT / "my_robot_package" / "assistant_node.py").read_text()
    ast.parse(source)
    assert '"navigate_to_room"' in source
    assert '"/api/navigate_room"' in source
    assert '"/api/start_follow"' in source
    assert "/assistant/text_query" in source
    assert "/assistant/text_response" in source
