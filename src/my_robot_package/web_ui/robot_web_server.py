import os
import subprocess
import signal
import time
import threading
from flask import Flask, jsonify, request

# Serve static files directly from the web_ui directory we created
app = Flask(__name__, static_folder='/home/harsh/ros2_ws/src/my_robot_package/web_ui', static_url_path='')

ros_process = None
rosbridge_process = None

def kill_process(proc):
    if proc is not None and proc.poll() is None:
        try:
            # Send SIGINT (Ctrl+C equivalent) for graceful ROS 2 shutdown
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            time.sleep(2) # Give nodes a moment to shut down cleanly
        except Exception as e:
            print(f"Error killing process: {e}")
            
    # Force kill any lingering nodes that failed to shut down
    subprocess.run(["pkill", "-9", "-f", "imu_node|motor_driver|encoder_odom|wall_follower|bumper_node|ir_driver|ultra_sensor|ldlidar|foxglove|nav2|amcl"])

@app.route('/')
def index():
    return app.send_static_file('index.html')

@app.route('/api/wifi', methods=['POST'])
def set_wifi():
    data = request.json
    ssid = data.get('ssid')
    password = data.get('password')
    
    # Only pass the password argument if one was provided (fixes open networks and empty strings)
    cmd = ['sudo', 'nmcli', 'dev', 'wifi', 'connect', ssid]
    if password and len(password) > 0:
        cmd.extend(['password', password])
        
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode == 0:
        return jsonify({"status": "success", "message": f"Successfully connected to {ssid}."})
    
    error_output = result.stderr.strip() if result.stderr.strip() else result.stdout.strip()
    return jsonify({"status": "error", "message": error_output})

@app.route('/api/wifi_scan', methods=['GET'])
def scan_wifi():
    try:
        # Run nmcli to get the list of SSIDs without column headers
        result = subprocess.run(['sudo', 'nmcli', '-t', '-f', 'SSID', 'dev', 'wifi'], capture_output=True, text=True)
        if result.returncode == 0:
            ssids = [line.strip() for line in result.stdout.split('\n') if line.strip()]
            unique_ssids = sorted(list(set(ssids))) # Remove duplicate and empty SSIDs
            return jsonify({"status": "success", "networks": unique_ssids})
        return jsonify({"status": "error", "message": result.stderr})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/api/check_imu', methods=['GET'])
def check_imu():
    try:
        result = subprocess.run(['i2cdetect', '-y', '1'], capture_output=True, text=True)
        if '68' in result.stdout:
            return jsonify({"status": "success", "message": "IMU is working"})
        return jsonify({"status": "error", "message": "IMU not detected"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

def do_start_rosbridge():
    global rosbridge_process
    if rosbridge_process is None or rosbridge_process.poll() is not None:
        # Kill any orphaned rosbridge processes to avoid port 9090 conflicts
        subprocess.run(["pkill", "-9", "-f", "rosbridge"])
        subprocess.run(["fuser", "-k", "9090/tcp"], stderr=subprocess.DEVNULL) # Force free port 9090
        cmd = "source /home/harsh/ros2_ws/install/setup.bash && ros2 run rosbridge_server rosbridge_websocket"
        rosbridge_process = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)

def do_start_teleop():
    global ros_process
    kill_process(ros_process)
    # Run just the motor driver for boot-time teleop
    cmd = "source /home/harsh/ros2_ws/install/setup.bash && ros2 run my_robot_package motor_driver_node"
    ros_process = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)

@app.route('/api/start_rosbridge', methods=['POST'])
def start_rosbridge():
    do_start_rosbridge()
    return jsonify({"status": "success", "message": "Rosbridge started"})

@app.route('/api/start_mapping', methods=['POST'])
def start_mapping():
    global ros_process
    kill_process(ros_process)
    cmd = "source /home/harsh/ros2_ws/install/setup.bash && ros2 launch my_robot_package bringup_and_map.launch.py"
    ros_process = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
    return jsonify({"status": "success", "message": "Mapping background process started."})

@app.route('/api/start_navigation', methods=['POST'])
def start_navigation():
    global ros_process
    kill_process(ros_process)
    cmd = "source /home/harsh/ros2_ws/install/setup.bash && ros2 launch my_robot_package navigation.launch.py"
    ros_process = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
    return jsonify({"status": "success", "message": "Navigation background process started."})

@app.route('/api/start_teleop', methods=['POST'])
def start_teleop():
    do_start_teleop()
    return jsonify({"status": "success", "message": "Teleop Only mode started. Ready to drive!"})

@app.route('/api/stop', methods=['POST'])
def stop():
    global ros_process
    kill_process(ros_process)
    ros_process = None
    return jsonify({"status": "success", "message": "All ROS 2 launch processes stopped."})

def schedule_power_action(command):
    # Use a fully detached subprocess instead of a Python thread.
    # Redirecting I/O to DEVNULL ensures it doesn't hold the web server's network sockets open!
    full_command = f"sleep 3 && {command}"
    subprocess.Popen(
        full_command, 
        shell=True, 
        start_new_session=True, 
        close_fds=True,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL, 
        stderr=subprocess.DEVNULL
    )

@app.route('/api/reboot', methods=['POST'])
def reboot_robot():
    try:
        kill_process(ros_process)
        schedule_power_action("sudo systemctl reboot || sudo reboot")
        return jsonify({"status": "success", "message": "Rebooting robot..."})
    except Exception as e:
        return jsonify({"status": "error", "message": f"Server error: {str(e)}"})

@app.route('/api/shutdown', methods=['POST'])
def shutdown_robot():
    try:
        kill_process(ros_process)
        schedule_power_action("sudo systemctl poweroff || sudo shutdown now")
        return jsonify({"status": "success", "message": "Shutting down robot..."})
    except Exception as e:
        return jsonify({"status": "error", "message": f"Server error: {str(e)}"})

@app.route('/api/sleep', methods=['POST'])
def sleep_robot():
    try:
        kill_process(ros_process)
        schedule_power_action("sudo systemctl suspend")
        return jsonify({"status": "success", "message": "Putting robot to sleep..."})
    except Exception as e:
        return jsonify({"status": "error", "message": f"Server error: {str(e)}"})

if __name__ == '__main__':
    # Disable Wi-Fi power management to prevent connection drops when Bluetooth is active
    subprocess.run(['sudo', 'iw', 'dev', 'wlan0', 'set', 'power_save', 'off'], stderr=subprocess.DEVNULL)
    
    # Automatically start rosbridge so the UI's web socket can connect immediately
    do_start_rosbridge()
    # Automatically start the motor driver so teleop works instantly on boot
    do_start_teleop()
    app.run(host='0.0.0.0', port=8080)