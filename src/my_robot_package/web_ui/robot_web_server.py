#!/usr/bin/env python3
"""Local Bob app server and exclusive owner of one robot operating mode."""

import atexit
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import threading
import time

from flask import Flask, jsonify, request

try:
    from .semantic_rooms import RoomStore
except ImportError:  # Script execution from the installed web_ui directory.
    from semantic_rooms import RoomStore


STATIC_DIRECTORY = Path(__file__).resolve().parent
ROBOT_WORKSPACE = Path(
    os.environ.get("ROBOT_WS", "/home/harsh/ros2_ws")
).expanduser()
ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")
WORKSPACE_SETUP = ROBOT_WORKSPACE / "install" / "setup.bash"
MAPS_DIRECTORY = Path(
    os.environ.get(
        "ROBOT_MAPS_DIR",
        str(ROBOT_WORKSPACE / "src" / "my_robot_package" / "maps"),
    )
).expanduser()
ROOMS_FILE = Path(
    os.environ.get(
        "ROBOT_ROOMS_FILE",
        "~/.config/mapping-robot/rooms.json",
    )
).expanduser()
ENROLLMENT_FILE = Path(
    os.environ.get(
        "ROBOT_ENROLLMENT_FILE",
        "~/.config/mapping-robot/target_enrollment.json",
    )
).expanduser()

TRACKING_COMMANDS = {
    "start_follow",
    "keep_frame",
    "stop",
    "enroll_target",
    "clear_enrollment",
    "take_photo",
    "start_recording",
    "stop_recording",
}
MAP_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,47}$")

app = Flask(__name__, static_folder=str(STATIC_DIRECTORY), static_url_path="")
room_store = RoomStore(ROOMS_FILE)
ros_process = None
rosbridge_process = None
active_mode = "stopped"
active_map = None
mode_generation = 0
process_lock = threading.RLock()
room_lock = threading.Lock()
operation_lock = threading.Lock()
operation_state = {
    "tracking": {"state": "idle", "message": ""},
    "navigation": {"state": "idle", "message": ""},
    "map_save": {"state": "idle", "message": ""},
}


def _ros_shell(arguments):
    """Build a fixed ROS shell command with safely quoted arguments."""
    quoted_command = " ".join(shlex.quote(str(item)) for item in arguments)
    return (
        "source %s && source %s && exec %s"
        % (
            shlex.quote(str(ROS_SETUP)),
            shlex.quote(str(WORKSPACE_SETUP)),
            quoted_command,
        )
    )


def _ros_process(arguments):
    """Start a fixed ROS command after sourcing ROS and this workspace."""
    return subprocess.Popen(
        ["/bin/bash", "-lc", _ros_shell(arguments)],
        start_new_session=True,
    )


def _run_ros(arguments, timeout=10.0):
    """Run one bounded ROS CLI operation and capture its result."""
    return subprocess.run(
        ["/bin/bash", "-lc", _ros_shell(arguments)],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def _set_operation(name, state, message=""):
    with operation_lock:
        operation_state[name] = {"state": state, "message": message}


def _operation_snapshot():
    with operation_lock:
        return {name: dict(value) for name, value in operation_state.items()}


def stop_process(process, graceful_timeout=8.0):
    """Stop only a process group created and owned by this server."""
    if process is None or process.poll() is not None:
        return
    try:
        process_group = os.getpgid(process.pid)
        os.killpg(process_group, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=graceful_timeout)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process_group, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=2.0)
        return
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process_group, signal.SIGKILL)
        except ProcessLookupError:
            return
        process.wait(timeout=2.0)


def replace_robot_process(arguments, mode, map_name=None):
    """Atomically replace the current robot mode and return its generation."""
    global active_map, active_mode, mode_generation, ros_process
    with process_lock:
        stop_process(ros_process)
        mode_generation += 1
        ros_process = _ros_process(arguments)
        active_mode = mode
        active_map = map_name
        return mode_generation


def do_start_rosbridge():
    """Start the one persistent rosbridge process if it is not already alive."""
    global rosbridge_process
    with process_lock:
        if rosbridge_process is None or rosbridge_process.poll() is not None:
            rosbridge_process = _ros_process(
                ["ros2", "run", "rosbridge_server", "rosbridge_websocket"]
            )


def do_start_teleop():
    """Enter direct manual-drive mode."""
    return replace_robot_process(
        ["ros2", "launch", "my_robot_package", "manual_control.launch.py"],
        "teleop",
    )


def _start_tracking_mode(allow_hot=False):
    """Start the tracker in motionless IDLE and return its generation."""
    generation = replace_robot_process(
        [
            "ros2",
            "launch",
            "my_robot_package",
            "person_tracking.launch.py",
            "initial_mode:=idle",
            "allow_hot_operation:=%s" % ("true" if allow_hot else "false"),
        ],
        "tracking",
    )
    _set_operation("tracking", "idle", "Tracker is starting in idle mode.")
    return generation


def available_maps():
    """Return saved map YAML files that also reference an existing image."""
    maps = []
    for yaml_path in sorted(MAPS_DIRECTORY.glob("*.yaml")):
        try:
            image_line = next(
                line for line in yaml_path.read_text().splitlines()
                if line.strip().startswith("image:")
            )
            image_name = image_line.split(":", 1)[1].strip().strip("'\"")
        except (OSError, StopIteration):
            continue
        if (yaml_path.parent / image_name).is_file():
            maps.append(yaml_path.name)
    return maps


def resolve_map(map_name=None):
    """Resolve an allowlisted saved map name to its absolute YAML path."""
    maps = available_maps()
    requested = Path(str(map_name or "")).name
    if requested and requested in maps:
        return requested, MAPS_DIRECTORY / requested
    preferred = "my_house_map_0515.yaml"
    if not requested and preferred in maps:
        return preferred, MAPS_DIRECTORY / preferred
    if not requested and maps:
        return maps[0], MAPS_DIRECTORY / maps[0]
    raise ValueError("Select one of the available saved maps.")


def _pi_health():
    temperature = None
    throttled = None
    try:
        temperature = round(
            float(Path("/sys/class/thermal/thermal_zone0/temp").read_text())
            / 1000.0,
            1,
        )
    except (OSError, ValueError):
        pass
    try:
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
            timeout=1.0,
        )
        if result.returncode == 0:
            throttled = result.stdout.strip().split("=", 1)[-1]
    except (OSError, subprocess.TimeoutExpired):
        pass
    return temperature, throttled


def _enrollment_present():
    """Return whether a minimally valid local appearance profile is stored."""
    try:
        payload = json.loads(ENROLLMENT_FILE.read_text())
    except (OSError, json.JSONDecodeError):
        return False
    return (
        isinstance(payload, dict)
        and payload.get("version") == 1
        and payload.get("kind") == "hsv_clothing_histogram"
        and isinstance(payload.get("signature"), list)
        and bool(payload["signature"])
    )


def _publish_tracking_when_ready(command, generation):
    _set_operation("tracking", "starting", "Waiting for the camera tracker.")
    deadline = time.monotonic() + 45.0
    while time.monotonic() < deadline:
        with process_lock:
            if generation != mode_generation or active_mode != "tracking":
                _set_operation("tracking", "canceled", "Tracking mode changed.")
                return
        try:
            topics = _run_ros(["ros2", "topic", "list"], timeout=4.0)
        except (OSError, subprocess.TimeoutExpired):
            time.sleep(0.5)
            continue
        if topics.returncode == 0 and "/tracking/command" in topics.stdout.splitlines():
            try:
                result = _run_ros(
                    [
                        "ros2",
                        "topic",
                        "pub",
                        "--once",
                        "/tracking/command",
                        "std_msgs/msg/String",
                        json.dumps({"data": command}),
                    ],
                    timeout=12.0,
                )
            except (OSError, subprocess.TimeoutExpired) as error:
                _set_operation("tracking", "error", str(error))
                return
            if result.returncode == 0:
                _set_operation("tracking", "sent", "Command sent: %s" % command)
            else:
                detail = result.stderr.strip() or result.stdout.strip()
                _set_operation("tracking", "error", detail[-400:])
            return
        time.sleep(0.5)
    _set_operation("tracking", "error", "Tracker did not become ready in 45 seconds.")


def schedule_tracking_command(command, generation=None):
    """Publish a validated tracking command without blocking an HTTP request."""
    if command not in TRACKING_COMMANDS:
        raise ValueError("Unsupported tracking command.")
    with process_lock:
        selected_generation = mode_generation if generation is None else generation
    threading.Thread(
        target=_publish_tracking_when_ready,
        args=(command, selected_generation),
        daemon=True,
    ).start()


def _goal_message(goal):
    yaw = float(goal.get("yaw", 0.0))
    return {
        "pose": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {
                    "x": float(goal["x"]),
                    "y": float(goal["y"]),
                    "z": 0.0,
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": math.sin(yaw / 2.0),
                    "w": math.cos(yaw / 2.0),
                },
            },
        }
    }


def _send_navigation_goal_when_ready(goal, generation, label):
    _set_operation("navigation", "starting", "Starting navigation to %s." % label)
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        with process_lock:
            if generation != mode_generation or active_mode != "navigation":
                _set_operation("navigation", "canceled", "Navigation mode changed.")
                return
        try:
            actions = _run_ros(["ros2", "action", "list"], timeout=4.0)
        except (OSError, subprocess.TimeoutExpired):
            time.sleep(0.75)
            continue
        if actions.returncode == 0 and "/navigate_to_pose" in actions.stdout.splitlines():
            _set_operation("navigation", "navigating", "Going to %s." % label)
            try:
                result = _run_ros(
                    [
                        "ros2",
                        "action",
                        "send_goal",
                        "/navigate_to_pose",
                        "nav2_msgs/action/NavigateToPose",
                        json.dumps(_goal_message(goal)),
                        "--feedback",
                    ],
                    timeout=600.0,
                )
            except subprocess.TimeoutExpired:
                _set_operation("navigation", "error", "Navigation timed out.")
                return
            except OSError as error:
                _set_operation("navigation", "error", str(error))
                return
            if result.returncode == 0 and "SUCCEEDED" in result.stdout.upper():
                _set_operation("navigation", "succeeded", "Arrived at %s." % label)
            else:
                detail = result.stderr.strip() or result.stdout.strip()
                _set_operation("navigation", "failed", detail[-500:])
            return
        time.sleep(0.75)
    _set_operation("navigation", "error", "Nav2 did not become ready in 60 seconds.")


def schedule_navigation_goal(goal, generation, label):
    """Wait for Nav2 and send one goal in a background worker."""
    threading.Thread(
        target=_send_navigation_goal_when_ready,
        args=(goal, generation, label),
        daemon=True,
    ).start()


def _start_navigation(map_name=None):
    selected_name, map_path = resolve_map(map_name)
    generation = replace_robot_process(
        [
            "ros2",
            "launch",
            "my_robot_package",
            "navigation.launch.py",
            "map:=%s" % map_path,
        ],
        "navigation",
        selected_name,
    )
    return selected_name, generation


@app.route("/")
def index():
    return app.send_static_file("index.html")


@app.route("/api/status", methods=["GET"])
def robot_status():
    global active_mode
    with process_lock:
        if ros_process is None or ros_process.poll() is not None:
            active_mode = "stopped"
        mode = active_mode
        selected_map = active_map
    temperature, throttled = _pi_health()
    try:
        room_count = len(room_store.list(selected_map))
    except RuntimeError:
        room_count = 0
    return jsonify(
        {
            "status": "success",
            "mode": mode,
            "map": selected_map,
            "room_count": room_count,
            "subject_enrolled": _enrollment_present(),
            "pi_temp_c": temperature,
            "throttled": throttled,
            "operations": _operation_snapshot(),
        }
    )


@app.route("/api/maps", methods=["GET"])
def list_maps():
    return jsonify({"status": "success", "maps": available_maps()})


@app.route("/api/rooms", methods=["GET", "POST"])
def rooms():
    if request.method == "GET":
        map_name = request.args.get("map")
        try:
            with room_lock:
                values = room_store.list(map_name)
        except RuntimeError as error:
            return jsonify({"status": "error", "message": str(error)}), 500
        return jsonify({"status": "success", "rooms": values})
    try:
        with room_lock:
            room = room_store.upsert(
                request.get_json(silent=True) or {}, set(available_maps())
            )
    except (RuntimeError, ValueError) as error:
        return jsonify({"status": "error", "message": str(error)}), 400
    return jsonify({"status": "success", "room": room}), 201


@app.route("/api/rooms/<path:room_name>", methods=["DELETE"])
def delete_room(room_name):
    map_name = request.args.get("map", "")
    with room_lock:
        deleted = room_store.delete(map_name, room_name)
    if not deleted:
        return jsonify({"status": "error", "message": "Room not found."}), 404
    return jsonify({"status": "success", "message": "Room deleted."})


@app.route("/api/start_mapping", methods=["POST"])
def start_mapping():
    replace_robot_process(
        ["ros2", "launch", "my_robot_package", "bringup_and_map.launch.py"],
        "mapping",
    )
    return jsonify(
        {"status": "success", "mode": "mapping", "message": "Mapping started."}
    )


@app.route("/api/start_navigation", methods=["POST"])
def start_navigation():
    data = request.get_json(silent=True) or {}
    try:
        selected_map, _generation = _start_navigation(data.get("map"))
    except ValueError as error:
        return jsonify({"status": "error", "message": str(error)}), 400
    return jsonify(
        {
            "status": "success",
            "mode": "navigation",
            "map": selected_map,
            "message": "Navigation started with %s." % selected_map,
        }
    )


@app.route("/api/navigation_goal", methods=["POST"])
def navigation_goal():
    data = request.get_json(silent=True) or {}
    try:
        goal = {
            "x": float(data["x"]),
            "y": float(data["y"]),
            "yaw": float(data.get("yaw", 0.0)),
        }
    except (KeyError, TypeError, ValueError):
        return jsonify({"status": "error", "message": "Invalid map goal."}), 400
    if not all(math.isfinite(value) for value in goal.values()):
        return jsonify({"status": "error", "message": "Invalid map goal."}), 400
    with process_lock:
        if active_mode != "navigation":
            return jsonify(
                {"status": "error", "message": "Start navigation mode first."}
            ), 409
        generation = mode_generation
    schedule_navigation_goal(goal, generation, "selected map point")
    return jsonify({"status": "success", "message": "Navigation goal queued."})


@app.route("/api/navigate_room", methods=["POST"])
def navigate_room():
    data = request.get_json(silent=True) or {}
    room_name = str(data.get("room", "")).strip()
    map_hint = data.get("map")
    try:
        with room_lock:
            room = room_store.resolve(room_name, map_hint)
    except KeyError as error:
        return jsonify({"status": "error", "message": str(error.args[0])}), 404
    except (RuntimeError, ValueError) as error:
        return jsonify({"status": "error", "message": str(error)}), 409
    with process_lock:
        reuse_navigation = active_mode == "navigation" and active_map == room["map"]
        generation = mode_generation
    if not reuse_navigation:
        try:
            _selected_map, generation = _start_navigation(room["map"])
        except ValueError as error:
            return jsonify({"status": "error", "message": str(error)}), 400
    schedule_navigation_goal(room["goal"], generation, room["name"])
    return jsonify(
        {
            "status": "success",
            "mode": "navigation",
            "room": room,
            "message": "Going to %s." % room["name"],
        }
    )


@app.route("/api/start_tracking", methods=["POST"])
def start_tracking():
    data = request.get_json(silent=True) or {}
    allow_hot = bool(data.get("allow_hot_operation", False))
    generation = _start_tracking_mode(allow_hot)
    return jsonify(
        {
            "status": "success",
            "mode": "tracking",
            "generation": generation,
            "message": "Tracking started in idle mode. Enroll before following.",
        }
    )


@app.route("/api/start_follow", methods=["POST"])
def start_follow():
    data = request.get_json(silent=True) or {}
    allow_hot = bool(data.get("allow_hot_operation", False))
    if not _enrollment_present():
        return jsonify(
            {
                "status": "error",
                "message": (
                    "Enroll yourself in Vision mode before starting Follow."
                ),
            }
        ), 409
    with process_lock:
        already_tracking = (
            active_mode == "tracking"
            and ros_process is not None
            and ros_process.poll() is None
        )
        generation = mode_generation
    if not already_tracking:
        generation = _start_tracking_mode(allow_hot)
    schedule_tracking_command("start_follow", generation)
    return jsonify(
        {
            "status": "success",
            "mode": "tracking",
            "hot_override": allow_hot,
            "message": "Follow requested; enrollment and safety gates must pass.",
        }
    )


@app.route("/api/tracking_command", methods=["POST"])
def tracking_command():
    data = request.get_json(silent=True) or {}
    command = str(data.get("command", "")).strip().lower()
    if command not in TRACKING_COMMANDS:
        return jsonify({"status": "error", "message": "Unsupported command."}), 400
    if command == "keep_frame" and not _enrollment_present():
        return jsonify(
            {
                "status": "error",
                "message": "Enroll yourself before enabling camera motion.",
            }
        ), 409
    start_if_needed = bool(data.get("start_if_needed", False))
    allow_hot = bool(data.get("allow_hot_operation", False))
    with process_lock:
        tracker_ready = (
            active_mode == "tracking"
            and ros_process is not None
            and ros_process.poll() is None
        )
        generation = mode_generation
    if not tracker_ready:
        if not start_if_needed:
            return jsonify(
                {"status": "error", "message": "Start tracking mode first."}
            ), 409
        generation = _start_tracking_mode(allow_hot)
    schedule_tracking_command(command, generation)
    return jsonify({"status": "success", "message": "Command queued: %s" % command})


@app.route("/api/start_teleop", methods=["POST"])
def start_teleop():
    do_start_teleop()
    return jsonify(
        {"status": "success", "mode": "teleop", "message": "Teleop started."}
    )


@app.route("/api/save_map", methods=["POST"])
def save_map():
    data = request.get_json(silent=True) or {}
    name = str(data.get("name", "")).strip()
    if not MAP_NAME_PATTERN.fullmatch(name):
        return jsonify(
            {"status": "error", "message": "Use letters, numbers, _ or -."}
        ), 400
    with process_lock:
        if active_mode != "mapping":
            return jsonify(
                {"status": "error", "message": "Start mapping mode first."}
            ), 409
    prefix = MAPS_DIRECTORY / name

    def worker():
        _set_operation("map_save", "saving", "Saving %s." % name)
        try:
            result = _run_ros(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", prefix],
                timeout=90.0,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            _set_operation("map_save", "error", str(error))
            return
        if result.returncode == 0:
            _set_operation("map_save", "saved", "Saved %s.yaml." % name)
        else:
            detail = result.stderr.strip() or result.stdout.strip()
            _set_operation("map_save", "error", detail[-500:])

    threading.Thread(target=worker, daemon=True).start()
    return jsonify({"status": "success", "message": "Map save started."})


@app.route("/api/stop", methods=["POST"])
def stop():
    global active_map, active_mode, mode_generation, ros_process
    with process_lock:
        stop_process(ros_process)
        ros_process = None
        active_mode = "stopped"
        active_map = None
        mode_generation += 1
    return jsonify(
        {"status": "success", "mode": "stopped", "message": "Robot stopped."}
    )


@app.route("/api/check_imu", methods=["GET"])
def check_imu():
    try:
        result = subprocess.run(
            ["i2cget", "-y", "1", "0x68", "0x75"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return jsonify({"status": "error", "message": str(error)}), 500
    if result.returncode == 0 and result.stdout.strip().lower() == "0x68":
        return jsonify({"status": "success", "message": "IMU responded at 0x68."})
    return jsonify({"status": "error", "message": "IMU did not respond."}), 503


@app.route("/api/start_rosbridge", methods=["POST"])
def start_rosbridge():
    do_start_rosbridge()
    return jsonify({"status": "success", "message": "Rosbridge started."})


@app.route("/api/wifi", methods=["POST"])
def set_wifi():
    data = request.get_json(silent=True) or {}
    ssid = str(data.get("ssid", "")).strip()
    password = str(data.get("password", ""))
    if not ssid:
        return jsonify({"status": "error", "message": "SSID is required."}), 400
    command = ["sudo", "nmcli", "dev", "wifi", "connect", ssid]
    if password:
        command.extend(["password", password])
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        return jsonify({"status": "error", "message": "Wi-Fi timed out."}), 504
    if result.returncode == 0:
        return jsonify({"status": "success", "message": "Connected to %s." % ssid})
    detail = result.stderr.strip() or result.stdout.strip()
    return jsonify({"status": "error", "message": detail}), 500


@app.route("/api/wifi_scan", methods=["GET"])
def scan_wifi():
    try:
        result = subprocess.run(
            ["sudo", "nmcli", "-t", "-f", "SSID", "dev", "wifi"],
            capture_output=True,
            text=True,
            timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return jsonify({"status": "error", "message": str(error)}), 500
    if result.returncode == 0:
        networks = sorted(
            {line.strip() for line in result.stdout.splitlines() if line.strip()}
        )
        return jsonify({"status": "success", "networks": networks})
    return jsonify({"status": "error", "message": result.stderr.strip()}), 500


def schedule_power_action(command):
    """Run one fixed power command after the HTTP response is sent."""
    def delayed_action():
        time.sleep(3)
        subprocess.Popen(
            command,
            start_new_session=True,
            close_fds=True,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    threading.Thread(target=delayed_action, daemon=True).start()


def _stop_before_power_action(command, message):
    stop()
    schedule_power_action(command)
    return jsonify({"status": "success", "message": message})


@app.route("/api/reboot", methods=["POST"])
def reboot_robot():
    return _stop_before_power_action(
        ["sudo", "systemctl", "reboot"], "Rebooting robot..."
    )


@app.route("/api/shutdown", methods=["POST"])
def shutdown_robot():
    return _stop_before_power_action(
        ["sudo", "systemctl", "poweroff"], "Shutting down robot..."
    )


@app.route("/api/sleep", methods=["POST"])
def sleep_robot():
    return _stop_before_power_action(
        ["sudo", "systemctl", "suspend"], "Putting robot to sleep..."
    )


def cleanup():
    global active_map, active_mode, ros_process, rosbridge_process
    with process_lock:
        stop_process(ros_process)
        stop_process(rosbridge_process)
        ros_process = None
        rosbridge_process = None
        active_mode = "stopped"
        active_map = None


if __name__ == "__main__":
    atexit.register(cleanup)
    subprocess.run(
        ["sudo", "iw", "dev", "wlan0", "set", "power_save", "off"],
        stderr=subprocess.DEVNULL,
        check=False,
    )
    do_start_rosbridge()
    # Keep the drive GPIO and motor command path disarmed until the user
    # explicitly selects Drive, Mapping, Navigation, or Vision in the app.
    # Rosbridge remains available so the UI and assistant can connect while
    # the robot is safely stopped.
    app.run(
        host=os.environ.get("ROBOT_WEB_HOST", "0.0.0.0"),
        port=int(os.environ.get("ROBOT_WEB_PORT", "8080")),
        threaded=True,
    )
