# Mapping Robot

ROS 2 Jazzy workspace for a Raspberry Pi companion robot with four mutually
exclusive operating modes:

1. **Teleop** — manually drive the base.
2. **Mapping** — build an unknown indoor or outdoor map with LiDAR, SLAM
   Toolbox, and the autonomous wall follower.
3. **Navigation** — localize in a saved map and drive from point A to point B
   with Nav2.
4. **Person tracking** — use OAK-D Lite spatial vision to follow a person,
   keep them framed, and capture photos or video indoors or outdoors.

Only one mode should own the motor driver at a time. The dashboard switches
modes by gracefully stopping the old launch process before starting the next.

## Workspace setup on the Pi

This repository is a complete colcon workspace snapshot: ROS packages live in
`src/`. Clone the requested branch with its pinned LiDAR and exploration
submodules:

```bash
git clone --recurse-submodules \
  --branch pi-snapshot-20260731 \
  https://github.com/harshcshah97-collab/mapping-robot.git \
  /home/harsh/ros2_ws
cd /home/harsh/ros2_ws
git submodule update --init --recursive
```

Install system and Python dependencies. DepthAI 3 is required by the current
camera pipeline:

```bash
sudo apt-get update
sudo apt-get install \
  espeak-ng i2c-tools mpg123 python3-flask python3-gpiozero \
  python3-opencv python3-pyaudio python3-smbus

python3 -m pip install \
  "depthai>=3.6,<4" openai SpeechRecognition ddgs bless

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
./start_build.sh
source install/setup.bash
```

`start_build.sh` applies the tracked, idempotent LiDAR pthread compatibility
patch in `patches/` because the pinned upstream driver omits that include.

The first OAK-D AI run needs internet access to download the selected Luxonis
model. Later runs can use its local cache.

## Frame and topic contract

The active stack uses this TF chain:

```text
map -> odom -> base_footprint -> base_link -> sensor frames
```

- SLAM Toolbox or AMCL owns `map -> odom`.
- `robot_localization` owns `odom -> base_footprint`.
- `robot_state_publisher` owns the fixed sensor transforms.
- Raw encoders publish `/odom` but do not publish TF in the launch files.
- The LD19 publishes `/scan` in `base_laser`.
- Full robot launches send behavior commands through `twist_mux`. Existing
  dashboard teleop remains `/cmd_vel`; tracking uses `/cmd_vel/tracking`, Nav2
  uses `/cmd_vel/navigation`, and mapping uses `/cmd_vel/mapping`. The motor
  consumes only the selected `/cmd_vel_out` and stops after 0.6 seconds without
  a fresh command.
- Manual teleop has the highest arbitration priority, followed by tracking,
  Nav2, and mapping. The bumper `/safety/stop` still goes directly to the motor
  driver and bypasses arbitration.

## Mapping

```bash
./start_mapping.sh
# or
ros2 launch my_robot_package bringup_and_map.launch.py
```

Mapping starts LiDAR, encoders, IMU/EKF, ultrasonic and IR sensors, SLAM
Toolbox, the motor driver, and the wall follower. The wall follower reads the
three bumpers directly, so `bumper_node` is intentionally not launched in this
mode; two processes must not own the same GPIO switches.

To add OAK-D viewing, person detections, and recording without letting the
camera drive:

```bash
ros2 launch my_robot_package bringup_and_map.launch.py \
  enable_oakd_perception:=true
```

The LiDAR remains the SLAM sensor. Save a completed map with the normal SLAM
Toolbox map-saving workflow.

### Frontier exploration

The original wall follower remains the default because it matches the known
working mapping flow. After measuring the base geometry and marking the
required checks in `config/calibration_status.yaml`, start Nav2 frontier
exploration instead:

```bash
ros2 launch my_robot_package bringup_and_map.launch.py \
  enable_frontier_exploration:=true
```

This starts `explore_lite` plus the Nav2 planner/controller while SLAM Toolbox
continues building `/map`. The launch makes the wall follower mutually
exclusive with frontier exploration. If critical calibration flags are still
false, it refuses to start autonomous motion and lists the missing checks.
Rebuild the workspace after changing an installed calibration file.

## Saved-map navigation

```bash
ros2 launch my_robot_package navigation.launch.py
```

Navigation now starts its own LD19 by default because AMCL and both Nav2
costmaps consume `/scan`. Disable it only when another process already owns the
sensor:

```bash
ros2 launch my_robot_package navigation.launch.py enable_lidar:=false
```

The default map is the installed `maps/my_house_map_0515.yaml`. Select another
valid map without editing the launch file:

```bash
ros2 launch my_robot_package navigation.launch.py \
  map:=/absolute/path/to/map.yaml
```

Set the initial pose before sending a goal. The dashboard sends goals to the
Nav2 `/navigate_to_pose` action, not to an unconsumed `/goal_pose` topic.

Optional camera intelligence remains perception-only while Nav2 owns motion:

```bash
ros2 launch my_robot_package navigation.launch.py \
  enable_oakd_perception:=true
```

## Person tracking and videography

Tracking does not require GPS, AMCL, or a saved map, so it works indoors and
outdoors:

```bash
ros2 launch my_robot_package person_tracking.launch.py
```

It starts idle: the camera detects people but the robot will not move until it
receives a command. Available commands on `/tracking/command` are:

```text
start_follow     turn toward the selected person, then approach
keep_frame       rotate in place to keep the person centered
enroll_target    remember the visible person's clothing-color appearance
clear_enrollment forget the saved subject appearance
stop             stop and return to idle
take_photo       save the latest RGB frame
start_recording  start an MP4 recording
stop_recording   finish and save the MP4 recording
```

Example:

```bash
ros2 topic pub --once /tracking/command std_msgs/msg/String \
  "{data: start_follow}"
```

Status is JSON on `/tracking/status`, events are on `/tracking/event`, and the
preview is `/oakd/color/image_raw`. Photos and video default to
`~/robot_recordings`.

Enrollment survives a restart in
`~/.config/mapping-robot/target_enrollment.json`. It is a lightweight HSV
clothing-color signature, not face recognition or a biometric identity. It
helps recover from a changed DepthAI track ID, but similar clothing or major
lighting changes can still confuse it. Clear and re-enroll when clothing or
lighting changes.

The recommended boot service runs Bob separately, so tracking defaults to
`enable_assistant:=false`. For a one-off run without that service:

```bash
ros2 launch my_robot_package person_tracking.launch.py \
  enable_assistant:=true
```

The controller stops if the target is lost, LiDAR becomes stale, an obstacle
is too close, or a bumper is pressed.

## Bob voice assistant at boot

Bob remains a first-class node. It uses the original `gpt-4o` default for text
and camera questions, the OpenAI speech API for replies, and publishes tracking
commands on `/assistant/command`. General AI-generated shell execution is
disabled by default.

Keep the API key outside Git. Create its systemd environment file:

```bash
mkdir -p /home/harsh/.config/mapping-robot
printf 'OPENAI_API_KEY=replace-with-your-key\n' \
  > /home/harsh/.config/mapping-robot/assistant.env
chmod 600 /home/harsh/.config/mapping-robot/assistant.env

sudo cp /home/harsh/ros2_ws/src/my_robot_package/web_ui/robot_assistant.service \
  /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot_assistant.service
```

For a foreground diagnostic instead:

```bash
export OPENAI_API_KEY="your-key-from-a-secret-store"
./start_assistant.sh
```

## Dashboard and BLE startup

The web server manages only the ROS process groups it created. It no longer
uses broad `pkill -9` commands that can kill unrelated or manually launched
nodes.

```bash
sudo cp /home/harsh/ros2_ws/src/my_robot_package/web_ui/robot_web.service \
  /home/harsh/ros2_ws/src/my_robot_package/web_ui/robot_ble.service \
  /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot_web.service robot_ble.service
```

Open `http://ROBOT_IP:8080`. The browser dependencies are currently loaded
from pinned CDN versions, so a completely offline deployment should vendor
those assets locally.

Security warning: the dashboard, BLE characteristics, and rosbridge endpoint
do not yet authenticate clients. Run them only on a trusted private robot
network or behind a firewall/VPN. Any device that can reach these ports may be
able to command motion or power actions.

## OAK-D depth in Nav2

The tracker can now publish a decimated `/oakd/depth/points` `PointCloud2`, and
both Nav2 costmaps have an OAK-D obstacle source. Normal launches keep it off
until two explicit gates pass:

1. the physical OAK-D mount transform is measured and the matching URDF joint
   is updated; and
2. point-cloud rate and Raspberry Pi temperature pass a hardware benchmark.

After measuring the mount, set `oakd_mount_transform_measured: true` in
`config/calibration_status.yaml` and set `oakd_extrinsics_calibrated: true` for
both camera node names in `config/hardware_calibration.yaml`. Build, then run a
motor-free benchmark:

```bash
# Terminal 1
ros2 launch my_robot_package oakd_depth_benchmark.launch.py

# Terminal 2: target at least 5 Hz and keep peak CPU temperature below 75 C
ros2 run my_robot_package perception_benchmark \
  --duration 120 --minimum-cloud-hz 5 --maximum-temperature-c 75
```

If it passes, record `oakd_depth_benchmark_passed: true` in both calibration
files. OAK-D obstacles can then be enabled for saved-map navigation:

```bash
ros2 launch my_robot_package navigation.launch.py \
  enable_oakd_depth_obstacles:=true
```

They can also augment the optional Nav2 costmaps during frontier mapping:

```bash
ros2 launch my_robot_package bringup_and_map.launch.py \
  enable_frontier_exploration:=true \
  enable_oakd_depth_obstacles:=true
```

LiDAR remains the primary SLAM and localization sensor. OAK-D depth improves
near-field obstacle awareness; it is not fed into SLAM Toolbox as a replacement
for the planar laser scan.

## Physical calibration and PWM control

The motor driver now controls the left and right wheels independently with PWM,
so Nav2 and tracking can request curves instead of only full-speed straight or
in-place motion. Open-loop PWM is the safe default. Encoder PI speed feedback
is present but is deliberately disabled until it is tuned on the actual robot.

The provisional values are centralized in
`config/hardware_calibration.yaml`; completion flags are in
`config/calibration_status.yaml`. Before changing a flag:

1. Raise the wheels and verify that positive left/right commands rotate both
   wheels forward and that `/safety/stop` immediately stops them.
2. Mark each tire, roll it several revolutions, and measure effective wheel
   circumference under load. Update `wheel_diameter` and `ticks_per_rev`.
3. Measure center-to-center wheel spacing and tune it with repeated 360-degree
   turns. Keep motor and encoder separation values identical.
4. Time several straight runs to determine the real maximum wheel speed, then
   update `max_wheel_speed_mps` and the minimum PWM that reliably starts each
   wheel.
5. Measure the farthest front, rear, left, and right extents, including bumpers
   and protruding sensors. Replace the provisional Nav2 footprint polygon.
6. Measure the OAK-D translation and rotation from `base_link`, then update
   `oakd_joint` in the URDF before enabling its depth gate.
7. Leave the robot motionless on a level surface, re-measure MPU6050 gyro bias,
   and verify the physical IMU mounting rotation.

Rebuild the workspace after editing either calibration file or the URDF so the
installed launch configuration receives the new measurements.

Check configuration consistency at any time:

```bash
ros2 run my_robot_package calibration_check
ros2 run my_robot_package calibration_check --strict
```

Only after raised-wheel and floor tuning pass should
`wheel_calibration_confirmed` and `closed_loop_enabled` be set true. Start with
low `kp`, add only enough `ki` to remove steady speed mismatch, and re-test the
watchdog and bumper stop after every tuning change.

## Known motion limitation: orbiting

Continuous cinematic orbiting remains disabled. Independent PWM wheel control
now permits smooth curves, but a differential-drive base cannot move sideways;
a fixed forward camera therefore cannot stay aimed at the center of a true
circle while the chassis travels tangentially.

A robust orbit needs one of:

- a camera pan gimbal combined with the new PWM wheel control;
- an omnidirectional base; or
- a slow Nav2 waypoint orbit that stops and turns toward the subject at each
  filming position.

Until then, the software rejects orbit commands instead of pretending a curved
forward drive is a subject-centred orbit.

## Verification

Run host-side static and pure-logic tests from the package directory:

```bash
cd /home/harsh/ros2_ws/src/my_robot_package
PYTHONPATH="$PWD" pytest -q
```

After every hardware or GPIO change, test with the drive wheels raised. On the
Pi, verify one mode at a time with `ros2 node list`, `ros2 topic hz /scan`,
`ros2 topic hz /odom`, `ros2 topic echo /tracking/status`, and the TF tree.
