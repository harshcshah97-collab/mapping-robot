# Repository and File Guide

[Start here](START_HERE.md) |
[Recommissioning manual](RECOMMISSIONING_MANUAL.md) |
[Wiring](WIRING_AND_POWER.md) |
[Nodes and data flow](NODES_AND_DATA_FLOW.md)

This repository is a complete ROS 2 colcon workspace snapshot. The expected Pi
path is `/home/harsh/ros2_ws`. Source is under `src/`; `build/`, `install/`, and
`log/` are generated locally and intentionally excluded from Git.

## Repository layout

```text
mapping-robot/
|-- README.md                         Current feature and operating overview
|-- docs/                             Long-term recommissioning documentation
|-- start_build.sh                    Apply LiDAR patch and build workspace
|-- start_mapping.sh                  Foreground mapping convenience wrapper
|-- start_assistant.sh                Foreground Bob assistant wrapper
|-- patches/                          Pinned third-party compatibility patch
|-- src/
|   |-- my_robot_package/             Project-owned ROS/Python/UI package
|   |-- ldlidar_stl_ros2/             Pinned Git submodule: LD19 driver
|   `-- m-explore-ros2/               Pinned Git submodule: frontier exploration
`-- cad/                              Local-only; intentionally ignored by Git
    |-- chassis_v2/                   V2 parametric CAD and measurements
    `-- chassis_v3_tall_compact/      V3 provisional compact/tall CAD
```

The `cad/` tree exists in the local archive checkout but is intentionally not
uploaded to GitHub. A fresh GitHub clone will not contain it.

## Root files

| File | Use it for | Do not use it for |
| --- | --- | --- |
| `README.md` | feature summary, common launch examples, calibration overview | the only recovery record; follow the main manual |
| `docs/RECOMMISSIONING_MANUAL.md` | start-to-finish archive and recovery procedure | storing live secrets |
| `docs/WIRING_AND_POWER.md` | GPIO map, signal diagram, power verification gates | assuming unrecorded power polarity/rating |
| `docs/NODES_AND_DATA_FLOW.md` | node/topic/service/mode architecture | replacing live `ros2 node info` checks |
| `docs/CREDENTIALS.template.md` | fields and locations for a private recovery record | entering secrets in Git |
| `start_build.sh` | source Jazzy, apply the pinned LD19 pthread patch, run `colcon build --symlink-install --packages-up-to my_robot_package` | installing OS packages |
| `start_mapping.sh` | source ROS/workspace and launch normal mapping | navigation, teleop, or tracking |
| `start_assistant.sh` | source ROS/workspace, configure user audio, require API key, start Bob with shell commands disabled | storing the API key |
| `switch_test.py` | legacy development experiment retained in snapshot | normal production startup |
| `.gitmodules` | exact upstream URLs for LD19 and exploration source | editing submodule contents without an intentional update |
| `patches/ldlidar-pthread.patch` | reproducible compile fix for pinned LD19 source | applying to an arbitrary newer driver revision |

## Package metadata

| File | Purpose |
| --- | --- |
| `src/my_robot_package/package.xml` | ROS package dependencies and package identity |
| `src/my_robot_package/setup.py` | Python package, data-file install rules, and `ros2 run` executables |
| `src/my_robot_package/setup.cfg` | Python executable install location |
| `src/my_robot_package/resource/my_robot_package` | ament index marker |

When adding a launch/config/map/UI file, ensure `setup.py` installs its file
pattern. When adding a normal Python node, add a `console_scripts` entry and the
matching ROS dependency in `package.xml`.

## Launch files

| File | Use |
| --- | --- |
| `launch/manual_control.launch.py` | minimal held-button/teleop stack with mux, motor watchdog, encoders, bumper |
| `launch/bringup_and_map.launch.py` | SLAM and legacy wall-follow mapping; optional frontier exploration and OAK perception |
| `launch/navigation.launch.py` | saved-map AMCL/Nav2 operation with selectable map and optional OAK perception/depth |
| `launch/person_tracking.launch.py` | production DepthAI 2.32 tracking stack, idle by default |
| `launch/ld19.launch.py` | LD19 product, `/dev/ttyUSB0`, 230400 baud, `/scan`, `base_laser` |
| `launch/oakd_depth_benchmark.launch.py` | motor-free OAK point-cloud publication for performance testing |
| `launch/depthai2_floor_follow_test.launch.py` | bounded and separately armed experimental floor-follow test; motion defaults off |

The web server starts the first four through the ROS CLI and enforces one active
mode. Launch a file manually only when the web server is not also managing a
mode, or its status/stop controls will not own that process.

## Configuration files

| File | Authority and edit rule |
| --- | --- |
| `config/hardware_calibration.yaml` | central motor pins/PWM, encoder geometry/directions, IMU bus/offsets, and OAK calibration gates; edit after physical measurement |
| `config/calibration_status.yaml` | boolean evidence record for completed physical checks; do not set a flag merely to bypass a launch gate |
| `config/person_tracking.yaml` | OAK blob/checksum, following speeds/distances, sensor and thermal gates, recording, enrollment |
| `config/twist_mux.yaml` | velocity source topics, priorities, and timeouts |
| `config/ekf.yaml` | raw encoder + IMU fusion and `odom -> base_footprint` TF ownership |
| `config/nav2_params.yaml` | AMCL, controller, planner, local/global costmaps, provisional footprint, obstacle sources |
| `config/mapper_params_online_async.yaml` | SLAM Toolbox mapping parameters |
| `config/explore.yaml` | `explore_lite` frontier behavior |
| `config/depthai2_floor_follow_test.yaml` | conservative limits for the bounded experimental test only |

Run `./start_build.sh` after changing installed config. Launch files consume the
copy under `install/`, not necessarily the source file, unless symlink behavior
applies to that file type.

## Project Python modules

| File | Role |
| --- | --- |
| `motor_driver_node.py` | GPIO motor output, watchdog, safety stop, optional encoder PI |
| `motor_logic.py` | pure wheel-target, limit, feed-forward, and PI calculations; host-testable |
| `encoder_odom_node.py` | quadrature counts to raw odometry |
| `imu_node.py` | MPU6050 acceleration/gyro publisher |
| `ultra_sensor_node.py` | HC-SR04 Range publisher |
| `ir_driver.py` | two digital FC-51 Range publishers |
| `bumper_node.py` | contact cloud and immediate stop Bool |
| `wall_follower.py` | legacy wall-follow mapping behavior and direct bumper handling |
| `person_follower_node.py` | OAK-D spatial tracking, safe motion, enrollment, media, point cloud |
| `tracking_logic.py` | pure follow/keep-frame motion calculation |
| `appearance_logic.py` | pure HSV appearance-signature and ROI helpers |
| `assistant_node.py` | Bob microphone/text/VLM/tools/speech ROS node |
| `assistant_logic.py` | pure assistant tracking-command translation |
| `calibration_check.py` | YAML consistency and calibration gate checker |
| `perception_benchmark.py` | point-cloud/scan/temperature benchmark |
| `check_vision.py` | minimal camera availability smoke test |
| `oakd_ai_test.py` | experimental model-zoo AI test |
| `depthai2_floor_follow_test_node.py` | bounded experimental test controller |
| `depthai2_floor_follow_logic.py` | pure command limits and test state logic |

Edit a pure `*_logic.py` module first when changing algorithms that can be
tested without ROS/hardware. Keep GPIO, ROS messages, and device lifetime in the
node modules.

## Web UI and services

| File | Role |
| --- | --- |
| `web_ui/robot_web_server.py` | Flask static server/API, process-group mode manager, rosbridge child, map/room actions, Wi-Fi/power endpoints |
| `web_ui/semantic_rooms.py` | validates polygons/goals and atomically stores named rooms |
| `web_ui/index.html` | app screens and controls |
| `web_ui/style.css` | responsive visual styling |
| `web_ui/main.js` | API calls, ROS WebSocket, map drawing, teleop, tracking and assistant interaction |
| `web_ui/manifest.webmanifest` | installable PWA metadata |
| `web_ui/service-worker.js` | local static-asset cache behavior |
| `web_ui/bob-icon.svg` | app icon |
| `web_ui/ble_wifi_scanner.py` | BLE advertisement, Wi-Fi provisioning, loopback mode commands |
| `web_ui/robot_web.service` | systemd unit for app/API and rosbridge |
| `web_ui/robot_ble.service` | systemd unit for BLE provisioning |
| `web_ui/robot_assistant.service` | systemd unit for Bob assistant and secret environment file |
| `web_ui/RobotBLEManager.swift` | iOS BLE manager reference source |
| `web_ui/ContentView.swift` | iOS tab/UI reference source |
| `web_ui/MapWebView.swift` | iOS web view wrapper for port 8080 |

The three Swift files are not a complete Xcode project. Preserve them as client
reference code or import them into a separately archived iOS project.

## Maps and local state

### Versioned maps

`src/my_robot_package/maps/` contains map YAML/PGM pairs. A YAML is offered in
the app only if its referenced image exists. `my_house_map_0515.yaml` is the
default. The `.posegraph`/`.data` pair is a serialized SLAM Toolbox map.

When adding a map, commit both the `.yaml` and image. Check that the YAML uses a
relative image name and records resolution, origin, occupied/free thresholds,
and negate correctly.

### Unversioned runtime state

| Path on Pi | Content | Backup handling |
| --- | --- | --- |
| `~/.config/mapping-robot/assistant.env` | OpenAI API key/overrides | secret; encrypted backup or password manager |
| `~/.config/mapping-robot/rooms.json` | named-room polygons and goals | private data backup; restore only with matching maps |
| `~/.config/mapping-robot/target_enrollment.json` | clothing-color signature | optional; re-enrollment is usually safer |
| `~/robot_recordings/` | photos and MP4 files | archive separately; can be large/private |
| `.venv-depthai2/` | rebuildable isolated Python environment | do not back up; recreate from pinned command |
| `build/`, `install/`, `log/` | generated colcon output | do not back up; rebuild |

## Tests

| Test group | What it protects |
| --- | --- |
| `test_motor_logic.py` | wheel kinematics, limits, feed-forward, PI behavior |
| `test_tracking_logic.py` | follow and framing command decisions |
| `test_appearance_logic.py` | enrollment/appearance math |
| `test_assistant_logic.py` | assistant command allowlist/translation |
| `test_calibration_check.py` | YAML consistency checks |
| `test_project_contracts.py` | launch/package/topic/frame/static contracts |
| `test_person_tracking_v2_contract.py` | isolated DepthAI 2 production guarantees |
| `test_depthai2_floor_follow_logic.py` | bounded test pure logic |
| `test_depthai2_floor_follow_contract.py` | floor-test safety/static contracts |
| `test_robot_app_contract.py` | API/mode-manager/UI contracts |
| `test_semantic_rooms.py` | room validation, storage, resolution |
| `test_flake8.py`, `test_pep257.py`, `test_copyright.py` | ROS package style checks |

Run host-side tests from the package source directory:

```bash
cd /home/harsh/ros2_ws/src/my_robot_package
PYTHONPATH="$PWD" pytest -q
```

On the Pi, also run:

```bash
cd /home/harsh/ros2_ws
./start_build.sh
colcon test --packages-select my_robot_package
colcon test-result --verbose
```

Host tests do not validate GPIO voltage, wheel direction, sensor geometry,
DepthAI/USB stability, stop distance, temperature under load, or mechanical
stability.

## CAD files

This section documents the separate local-only CAD archive. These files are
ignored by Git and are not part of the GitHub branch or recommissioning tag.

### V2

- `cad/chassis_v2/model.py`: geometry source.
- `cad/chassis_v2/dimensions.json`: design/measurement register.
- `cad/chassis_v2/MEASUREMENTS.md`: physical measurement worksheet.
- `cad/chassis_v2/cadkernel.py` and `exporters.py`: dependency-free CSG/export.
- `cad/chassis_v2/validate_mesh.py`: generated-mesh checks.
- `cad/chassis_v2/render_preview.py`: SVG preview renderer.
- `cad/chassis_v2/generated/`: STL/3MF/manifest/assembly outputs.

### V3 tall compact

- `cad/chassis_v3_tall_compact/model.py`: V3 geometry source, reusing V2 helpers.
- `dimensions.json`: V3 design register and unverified assumptions.
- `render_preview.py`: preview renderer.
- `generated/`: V3 STL/3MF/manifest/assembly outputs.

Both local CAD packages are provisional. In V3 only the fit coupon is released
for printing. Files named `DO_NOT_PRINT` or `DO_NOT_SLICE` are inspection
assemblies, not manufacturing output. Read each locally retained CAD README
before generating or printing.

## Typical change workflows

### Change a GPIO assignment

1. Change the relevant node default and `hardware_calibration.yaml` where it is
   centrally overridden.
2. Update `docs/WIRING_AND_POWER.md` and the physical cable label.
3. Add/update contract tests.
4. Build, test unpowered input behavior, then perform restrained raised-wheel
   validation.

### Change tracking behavior

1. Adjust pure decisions in `tracking_logic.py` or appearance math in
   `appearance_logic.py` with tests.
2. Adjust device/ROS behavior in `person_follower_node.py`.
3. Put deployed values in `person_tracking.yaml`.
4. Run all tests and a motor-free camera test, then a restrained floor test.

### Change UI mode behavior

1. Update endpoint/process ownership in `robot_web_server.py`.
2. Update browser interaction in `main.js` and markup/style as needed.
3. Update `test_robot_app_contract.py`.
4. Confirm STOP, disconnect, hidden-tab, mode replacement, and service restart
   behavior before restoring motor power.

### Change chassis geometry

1. Fill physical measurements first.
2. Update the selected chassis `model.py`; keep `dimensions.json` in sync.
3. Regenerate and validate.
4. Print only released coupons/adapters until all documented gates pass.
5. Measure the assembled robot and update URDF, footprint, wheel geometry, and
   speed/acceleration limits before autonomous motion.

## Files never to commit

- API keys, Wi-Fi passwords, Linux passwords, SSH private keys, access tokens.
- `assistant.env`, `.env`, or any `*.private.md` record.
- Runtime room/enrollment databases unless deliberately anonymized.
- Private recordings or photos.
- Generated ROS build/install/log directories and Python virtual environments.
- OS images containing credentials unless encrypted and stored outside GitHub.
