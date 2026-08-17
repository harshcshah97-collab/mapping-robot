let ros = null;
let viewer = null;
let gridClient = null;
let cmdVelTopic = null;
let trackingCommandTopic = null;
let activeRobotMode = 'unknown';

function currentRosTime() {
    const milliseconds = Date.now();
    return {
        sec: Math.floor(milliseconds / 1000),
        nanosec: (milliseconds % 1000) * 1000000
    };
}

// Automatically fill in the IP of the server if accessed remotely
document.getElementById('robot-ip').value = window.location.hostname || "127.0.0.1";

function connectROS() {
    const robot_ip = document.getElementById('robot-ip').value;
    const statusEl = document.getElementById('status');

    if (ros) { ros.close(); }

    statusEl.className = 'alert alert-warning text-center';
    statusEl.innerText = 'Connecting to ' + robot_ip + '...';

    ros = new ROSLIB.Ros({
        url: 'ws://' + robot_ip + ':9090'
    });

    ros.on('connection', () => {
        statusEl.className = 'alert alert-success text-center';
        statusEl.innerText = 'Connected to Robot (' + robot_ip + ')';
        setupROSInterfaces();
    });

    ros.on('error', (error) => {
        statusEl.className = 'alert alert-danger text-center';
        statusEl.innerText = 'Error connecting to Robot Websocket.';
    });

    ros.on('close', () => {
        statusEl.className = 'alert alert-warning text-center';
        statusEl.innerText = 'Connection to Robot Closed.';
    });
}

// Backend Launch Triggers
function triggerAction(action) {
    fetch('/api/' + action, { method: 'POST' })
        .then(res => res.json())
        .then(data => {
            if (data.mode) activeRobotMode = data.mode;
            alert(data.message);
            if (action === 'start_mapping' || action === 'start_navigation') {
                // Wait 5 seconds for nodes to spin up, then check sensors
                setTimeout(runSensorCheck, 5000);
            }
        })
        .catch(err => alert("Error triggering action: " + err));
}

function sendTrackingCommand(command) {
    if (!trackingCommandTopic || !ros || !ros.isConnected) {
        alert('Connect to ROS before sending a tracking command.');
        return;
    }
    if (activeRobotMode !== 'tracking') {
        alert('Start Tracking Mode before sending camera motion commands.');
        return;
    }
    trackingCommandTopic.publish(new ROSLIB.Message({ data: command }));
}

function runSensorCheck() {
    const overlay = document.getElementById('sensor-check-overlay');
    overlay.classList.remove('d-none');
    overlay.classList.add('d-flex');

    document.getElementById('check-imu').innerText = '⏳';
    document.getElementById('check-ultra').innerText = '⏳';
    document.getElementById('check-ir-left').innerText = '⏳';
    document.getElementById('check-ir-right').innerText = '⏳';

    let sensors = { ultra: false, irLeft: false, irRight: false };

    // Call our Python backend to run the physical i2cdetect test
    fetch('/api/check_imu')
        .then(res => res.json())
        .then(data => { document.getElementById('check-imu').innerText = data.status === 'success' ? '✅' : '❌'; })
        .catch(() => { document.getElementById('check-imu').innerText = '❌'; });

    // Temporarily subscribe to topics to verify nodes are alive and publishing
    if (ros && ros.isConnected) {
        let ultraTopic = new ROSLIB.Topic({ ros: ros, name: '/ultrasonic_distance', messageType: 'sensor_msgs/msg/Range' });
        let irLeftTopic = new ROSLIB.Topic({ ros: ros, name: '/ir/left', messageType: 'sensor_msgs/msg/Range' });
        let irRightTopic = new ROSLIB.Topic({ ros: ros, name: '/ir/right', messageType: 'sensor_msgs/msg/Range' });

        ultraTopic.subscribe(() => { sensors.ultra = true; document.getElementById('check-ultra').innerText = '✅'; ultraTopic.unsubscribe(); });
        irLeftTopic.subscribe(() => { sensors.irLeft = true; document.getElementById('check-ir-left').innerText = '✅'; irLeftTopic.unsubscribe(); });
        irRightTopic.subscribe(() => { sensors.irRight = true; document.getElementById('check-ir-right').innerText = '✅'; irRightTopic.unsubscribe(); });

        // Close the diagnostic overlay after giving sensors 5 seconds to respond
        setTimeout(() => {
            if (!sensors.ultra) { document.getElementById('check-ultra').innerText = '❌'; ultraTopic.unsubscribe(); }
            if (!sensors.irLeft) { document.getElementById('check-ir-left').innerText = '❌'; irLeftTopic.unsubscribe(); }
            if (!sensors.irRight) { document.getElementById('check-ir-right').innerText = '❌'; irRightTopic.unsubscribe(); }
            setTimeout(() => { overlay.classList.remove('d-flex'); overlay.classList.add('d-none'); }, 3000);
        }, 5000);
    } else {
        document.getElementById('check-ultra').innerText = '❌';
        document.getElementById('check-ir-left').innerText = '❌';
        document.getElementById('check-ir-right').innerText = '❌';
        setTimeout(() => {
            overlay.classList.remove('d-flex');
            overlay.classList.add('d-none');
        }, 3000);
    }
}

function scanWifi() {
    const ssidDropdown = document.getElementById('ssid');
    const scanBtn = document.getElementById('scanWifiBtn');
    if (!ssidDropdown) return;
    
    ssidDropdown.innerHTML = '<option value="">Scanning...</option>';
    if (scanBtn) {
        scanBtn.disabled = true;
        scanBtn.innerHTML = '<span class="spinner-border spinner-border-sm" role="status" aria-hidden="true"></span>';
    }
    
    fetch('/api/wifi_scan')
        .then(response => response.json())
        .then(data => {
            if (data.status === 'success') {
                ssidDropdown.innerHTML = '<option value="">Select a network...</option>';
                data.networks.forEach(ssid => {
                    const option = document.createElement('option');
                    option.value = ssid;
                    option.textContent = ssid;
                    ssidDropdown.appendChild(option);
                });
            } else {
                ssidDropdown.innerHTML = '<option value="">Error scanning</option>';
                console.error('Wi-Fi scan failed:', data.message);
            }
        })
        .catch(err => {
            ssidDropdown.innerHTML = '<option value="">Failed to reach server</option>';
            console.error('Error:', err);
        })
        .finally(() => {
            if (scanBtn) {
                scanBtn.disabled = false;
                scanBtn.innerHTML = 'Scan';
            }
        });
}

function changeWifi() {
    const ssid = document.getElementById('ssid').value;
    const password = document.getElementById('password').value;
    if (!ssid) return alert("Please enter a WiFi SSID");
    
    document.getElementById('wifiStatus').innerText = "Connecting... Your device may disconnect temporarily.";
    fetch('/api/wifi', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ssid, password })
    })
    .then(res => res.json())
    .then(data => {
        document.getElementById('wifiStatus').innerText = data.message;
    })
    .catch(err => {
        document.getElementById('wifiStatus').innerText = "Network dropped or Error: " + err;
    });
}

function powerAction(action) {
    // Confirm with the user before doing a destructive action
    if (!confirm(`Are you sure you want to ${action} the robot?`)) {
        return;
    }
    
    fetch(`/api/${action}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    })
    .then(response => response.json())
    .then(data => {
        alert(data.message);
    })
    .catch(error => {
        console.error(`Error executing ${action}:`, error);
        alert(`Failed to execute ${action}. Check your connection.`);
    });
}

function setupROSInterfaces() {
    // Map Visualization Setup
    if (!viewer) {
        viewer = new ROS2D.Viewer({
            divID: 'map',
            width: 640,
            height: 480
        });
    }
    
    if (gridClient) {
        viewer.scene.removeChild(gridClient.rootObject);
    }

    gridClient = new ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: viewer.scene,
        continuous: true
    });
    
    // Navigation Goal Status Subscription
    let goalStatusTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/navigate_to_pose/_action/status',
        messageType: 'action_msgs/msg/GoalStatusArray'
    });

    goalStatusTopic.subscribe((msg) => {
        if (msg.status_list && msg.status_list.length > 0) {
            let latest_status = msg.status_list[msg.status_list.length - 1].status;
            const statusEl = document.getElementById('goal-status');
            statusEl.classList.remove('d-none');
            if (latest_status === 1 || latest_status === 2) {
                statusEl.className = 'alert alert-info text-center mt-3'; statusEl.innerText = '🚀 Navigation in progress...';
            } else if (latest_status === 4) {
                statusEl.className = 'alert alert-success text-center mt-3'; statusEl.innerText = '✅ Goal Succeeded!';
            } else if (latest_status === 5 || latest_status === 6) {
                statusEl.className = 'alert alert-danger text-center mt-3'; statusEl.innerText = '❌ Goal Failed or Canceled.';
            }
        }
    });

    gridClient.on('change', () => {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });

    // Teleop Setup
    cmdVelTopic = new ROSLIB.Topic({ 
        ros: ros, 
        name: '/cmd_vel', 
        messageType: 'geometry_msgs/msg/Twist' 
    });

    trackingCommandTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/tracking/command',
        messageType: 'std_msgs/msg/String'
    });
    const trackingStatusTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/tracking/status',
        messageType: 'std_msgs/msg/String'
    });
    trackingStatusTopic.subscribe((message) => {
        const element = document.getElementById('tracking-status');
        if (element) element.innerText = message.data;
    });
}

// Map Interaction: Click and Drag to set Pose/Goal
let interactionMode = null;
let mouseDownPos = null;

function setInteractionMode(mode) {
    if (activeRobotMode !== 'navigation') {
        alert('Start Navigation mode before setting a pose or goal.');
        return;
    }
    interactionMode = mode;
    document.getElementById('btn-pose').classList.toggle('active', mode === 'pose');
    document.getElementById('btn-goal').classList.toggle('active', mode === 'goal');
}

window.onload = () => {
    // Periodically check if the viewer has initialized, then attach map mouse events
    const checkViewer = setInterval(() => {
        if (viewer && viewer.scene) {
            clearInterval(checkViewer);
            
            viewer.scene.addEventListener('stagemousedown', (event) => {
                if (!interactionMode) return;
                mouseDownPos = viewer.scene.globalToRos(event.stageX, event.stageY);
            });

            viewer.scene.addEventListener('stagemouseup', (event) => {
                if (!interactionMode || !mouseDownPos) return;
                let mouseUpPos = viewer.scene.globalToRos(event.stageX, event.stageY);
                
                // Calculate orientation quaternion from drag direction
                let dx = mouseUpPos.x - mouseDownPos.x;
                let dy = mouseUpPos.y - mouseDownPos.y;
                let theta = Math.atan2(dy, dx);
                let q = { x: 0.0, y: 0.0, z: Math.sin(theta / 2.0), w: Math.cos(theta / 2.0) };

                if (interactionMode === 'pose') {
                    let topic = new ROSLIB.Topic({ ros: ros, name: '/initialpose', messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped' });
                    let msg = new ROSLIB.Message({
                        header: { stamp: currentRosTime(), frame_id: 'map' },
                        pose: { pose: { position: { x: mouseDownPos.x, y: mouseDownPos.y, z: 0.0 }, orientation: q }, covariance: [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.068] }
                    });
                    topic.publish(msg);
                } else if (interactionMode === 'goal') {
                    // Nav2 accepts navigation requests through its action server,
                    // not by subscribing to a /goal_pose topic.
                    const goalId = 'web-nav-' + Date.now();
                    ros.callOnConnection({
                        op: 'send_action_goal',
                        id: goalId,
                        action: '/navigate_to_pose',
                        action_type: 'nav2_msgs/action/NavigateToPose',
                        feedback: true,
                        args: {
                            pose: {
                                header: { stamp: currentRosTime(), frame_id: 'map' },
                                pose: {
                                    position: { x: mouseDownPos.x, y: mouseDownPos.y, z: 0.0 },
                                    orientation: q
                                }
                            },
                            behavior_tree: ''
                        }
                    });
                }
                
                setInteractionMode(null);
                mouseDownPos = null;
            });
        }
    }, 500);

    // Teleop D-Pad Setup
    const btnUp = document.getElementById('btn-up');
    const btnDown = document.getElementById('btn-down');
    const btnLeft = document.getElementById('btn-left');
    const btnRight = document.getElementById('btn-right');
    const btnStop = document.getElementById('btn-stop');

    let moveInterval = null;

    function startMoving(linear, angular) {
        if (!cmdVelTopic) return;
        if (activeRobotMode !== 'teleop') {
            alert('Start Teleop mode before using the drive pad.');
            return;
        }
        if (moveInterval) clearInterval(moveInterval);
        const msg = new ROSLIB.Message({ linear: { x: linear, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: angular } });
        cmdVelTopic.publish(msg);
        // Publish continuously at 10Hz while held down to prevent timeout stopping
        moveInterval = setInterval(() => cmdVelTopic.publish(msg), 100);
    }

    function stopMoving() {
        if (!cmdVelTopic) return;
        if (moveInterval) clearInterval(moveInterval);
        moveInterval = null;
        cmdVelTopic.publish(new ROSLIB.Message({ linear: { x: 0.0, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.0 } }));
    }

    const addControl = (element, linear, angular) => {
        if (!element) return;
        element.addEventListener('mousedown', () => startMoving(linear, angular));
        element.addEventListener('mouseup', stopMoving);
        element.addEventListener('mouseleave', stopMoving);
        element.addEventListener('touchstart', (e) => { e.preventDefault(); startMoving(linear, angular); }, {passive: false});
        element.addEventListener('touchend', (e) => { e.preventDefault(); stopMoving(); }, {passive: false});
        element.addEventListener('touchcancel', stopMoving);
    };

    addControl(btnUp, 0.22, 0.0);
    addControl(btnDown, -0.22, 0.0);
    addControl(btnLeft, 0.0, 1.0);
    addControl(btnRight, 0.0, -1.0);
    
    if (btnStop) {
        btnStop.addEventListener('click', stopMoving);
        btnStop.addEventListener('touchstart', (e) => { e.preventDefault(); stopMoving(); }, {passive: false});
    }

    window.addEventListener('blur', stopMoving);
    document.addEventListener('visibilitychange', () => {
        if (document.hidden) stopMoving();
    });
    
    // Populate Wi-Fi dropdown and bind scan button
    scanWifi();
    const scanBtn = document.getElementById('scanWifiBtn');
    if (scanBtn) {
        scanBtn.addEventListener('click', scanWifi);
    }

    // Auto connect using the pre-filled hostname/IP
    fetch('/api/status')
        .then(response => response.json())
        .then(data => { if (data.mode) activeRobotMode = data.mode; })
        .catch(() => { activeRobotMode = 'unknown'; });
    connectROS();
};
