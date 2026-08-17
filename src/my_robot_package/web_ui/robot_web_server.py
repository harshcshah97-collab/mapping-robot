#!/usr/bin/env python3
"""Local dashboard server and owner for one managed ROS launch process."""

import atexit
import os
from pathlib import Path
import shlex
import signal
import subprocess
import threading
import time

from flask import Flask, jsonify, request


STATIC_DIRECTORY = Path(__file__).resolve().parent
ROBOT_WORKSPACE = Path(
    os.environ.get("ROBOT_WS", "/home/harsh/ros2_ws")
).expanduser()
ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")
WORKSPACE_SETUP = ROBOT_WORKSPACE / "install" / "setup.bash"

app = Flask(
    __name__, static_folder=str(STATIC_DIRECTORY), static_url_path=""
)

ros_process = None
rosbridge_process = None
active_mode = "stopped"
process_lock = threading.Lock()


def _ros_process(arguments):
    """Start a fixed ROS command after sourcing ROS and this workspace."""
    quoted_command = " ".join(shlex.quote(str(item)) for item in arguments)
    script = (
        f"source {shlex.quote(str(ROS_SETUP))} && "
        f"source {shlex.quote(str(WORKSPACE_SETUP))} && "
        f"exec {quoted_command}"
    )
    return subprocess.Popen(
        ["/bin/bash", "-lc", script],
        start_new_session=True,
    )


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


def replace_robot_process(arguments, mode):
    """Atomically replace the current robot mode with a new launch process."""
    global active_mode, ros_process
    with process_lock:
        stop_process(ros_process)
        ros_process = _ros_process(arguments)
        active_mode = mode


def do_start_rosbridge():
    global rosbridge_process
    with process_lock:
        if rosbridge_process is None or rosbridge_process.poll() is not None:
            rosbridge_process = _ros_process([
                "ros2", "run", "rosbridge_server", "rosbridge_websocket"
            ])


def do_start_teleop():
    replace_robot_process([
        "ros2", "run", "my_robot_package", "motor_driver_node"
    ], "teleop")


@app.route("/")
def index():
    return app.send_static_file("index.html")


@app.route("/api/status", methods=["GET"])
def robot_status():
    global active_mode, ros_process
    with process_lock:
        if ros_process is None or ros_process.poll() is not None:
            active_mode = "stopped"
        return jsonify({"status": "success", "mode": active_mode})


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
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=30
        )
    except subprocess.TimeoutExpired:
        return jsonify({"status": "error", "message": "Wi-Fi command timed out."}), 504

    if result.returncode == 0:
        return jsonify({
            "status": "success",
            "message": f"Successfully connected to {ssid}.",
        })
    error = result.stderr.strip() or result.stdout.strip()
    return jsonify({"status": "error", "message": error}), 500


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
        networks = sorted({
            line.strip() for line in result.stdout.splitlines() if line.strip()
        })
        return jsonify({"status": "success", "networks": networks})
    return jsonify({"status": "error", "message": result.stderr.strip()}), 500


@app.route("/api/check_imu", methods=["GET"])
def check_imu():
    try:
        result = subprocess.run(
            ["i2cdetect", "-y", "1"], capture_output=True, text=True, timeout=5
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return jsonify({"status": "error", "message": str(error)}), 500

    if result.returncode == 0 and "68" in result.stdout:
        return jsonify({"status": "success", "message": "IMU is working"})
    return jsonify({"status": "error", "message": "IMU not detected"}), 503


@app.route("/api/start_rosbridge", methods=["POST"])
def start_rosbridge():
    do_start_rosbridge()
    return jsonify({"status": "success", "message": "Rosbridge started"})


@app.route("/api/start_mapping", methods=["POST"])
def start_mapping():
    replace_robot_process([
        "ros2", "launch", "my_robot_package", "bringup_and_map.launch.py"
    ], "mapping")
    return jsonify({
        "status": "success", "mode": "mapping",
        "message": "Mapping mode started."
    })


@app.route("/api/start_navigation", methods=["POST"])
def start_navigation():
    replace_robot_process([
        "ros2", "launch", "my_robot_package", "navigation.launch.py"
    ], "navigation")
    return jsonify({
        "status": "success", "mode": "navigation",
        "message": "Navigation mode started."
    })


@app.route("/api/start_tracking", methods=["POST"])
def start_tracking():
    replace_robot_process([
        "ros2", "launch", "my_robot_package", "person_tracking.launch.py"
    ], "tracking")
    return jsonify({
        "status": "success", "mode": "tracking",
        "message": "Person tracking mode started."
    })


@app.route("/api/start_teleop", methods=["POST"])
def start_teleop():
    do_start_teleop()
    return jsonify({
        "status": "success", "mode": "teleop",
        "message": "Teleop mode started."
    })


@app.route("/api/stop", methods=["POST"])
def stop():
    global active_mode, ros_process
    with process_lock:
        stop_process(ros_process)
        ros_process = None
        active_mode = "stopped"
    return jsonify({
        "status": "success", "mode": "stopped",
        "message": "Robot mode stopped."
    })


def schedule_power_action(command):
    """Run one fixed power-management command after the HTTP reply is sent."""
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
    global active_mode, ros_process
    with process_lock:
        stop_process(ros_process)
        ros_process = None
        active_mode = "stopped"
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
    global active_mode, ros_process, rosbridge_process
    with process_lock:
        stop_process(ros_process)
        stop_process(rosbridge_process)
        ros_process = None
        rosbridge_process = None
        active_mode = "stopped"


if __name__ == "__main__":
    atexit.register(cleanup)
    subprocess.run(
        ["sudo", "iw", "dev", "wlan0", "set", "power_save", "off"],
        stderr=subprocess.DEVNULL,
        check=False,
    )
    do_start_rosbridge()
    do_start_teleop()
    app.run(
        host=os.environ.get("ROBOT_WEB_HOST", "0.0.0.0"),
        port=int(os.environ.get("ROBOT_WEB_PORT", "8080")),
        threaded=True,
    )
