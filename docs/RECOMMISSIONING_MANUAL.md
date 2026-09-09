# Bob Robot Recommissioning Manual

Archive date: 2026-09-09

Robot: Bob, Raspberry Pi 5 ROS 2 companion/mapping robot

Repository: `https://github.com/harshcshah97-collab/mapping-robot`

Archive branch: `codex/companion-robot-hardening`

Archive tag: `recommissioning-archive-2026-09-09`

This is the start-to-finish recovery runbook. Read it completely before applying
power. Follow the linked wiring and node references where a step calls for them.

## Documentation set

- [Wiring and power](WIRING_AND_POWER.md): exact GPIO map, USB connections,
  signal circuit, provisional power topology, connector labels, first power.
- [Nodes and data flow](NODES_AND_DATA_FLOW.md): every project node, external
  launch node, topic, mode, service, port, assistant, UI, and BLE flow.
- [Repository and file guide](FILE_GUIDE.md): directory structure, purpose of
  each project file, maps, tests, local-only CAD notes, and edit workflows.
- [Credential template](CREDENTIALS.template.md): where future access records
  belong. The Git copy deliberately contains no secret values.
- [Main README](../README.md): shorter operating and calibration overview.

## 1. What this archive contains

The codebase implements four mutually exclusive motion modes:

1. Drive: manual held-button teleoperation.
2. Mapping: LD19 + SLAM Toolbox with legacy wall following, or calibration-gated
   Nav2 frontier exploration.
3. Navigation: AMCL + Nav2 on a selected saved map.
4. Vision: OAK-D Lite person detection, enrollment, following, framing, photos,
   and video.

Bob's voice/text assistant and the Flask web app can remain available while the
drive stack is stopped. The app is the intended mode owner: it stops its current
ROS process group before starting another one.

The GitHub repository contains ROS source, launch/configuration, saved maps, web
and BLE software, tests, and third-party submodule pins. The provisional V2/V3
CAD directory is intentionally excluded from GitHub and remains only in the
local archive checkout. Git also does not contain Linux passwords, Wi-Fi
passwords, private SSH keys, OpenAI API keys, runtime room/enrollment state,
recordings, the Pi OS image, or the production MobileNet blob.

## 2. Known recovery facts

### GitHub

- HTTPS URL: `https://github.com/harshcshah97-collab/mapping-robot.git`
- SSH URL: `git@github.com:harshcshah97-collab/mapping-robot.git`
- Long-term snapshot: tag `recommissioning-archive-2026-09-09`
- Working archive branch: `codex/companion-robot-hardening`
- Previous Pi snapshot branch: `pi-snapshot-20260731`
- Required submodules:
  - `src/ldlidar_stl_ros2` at the commit recorded by Git.
  - `src/m-explore-ros2` at the commit recorded by Git.

The tag is the reproducible starting point. Create a new branch from it rather
than committing new development directly to the tag state.

### Robot network and SSH

- Expected Linux user: `harsh`
- Preferred last-known mDNS hostname: `harsh-rpi.local`
- Last-known address candidates: `10.21.10.17`, `192.168.3.2`,
  `192.168.1.216`, `192.168.1.68`
- Expected workspace: `/home/harsh/ros2_ws`

These values came from local SSH records and source paths. They were not
reachable for live verification on the archive date. DHCP addresses are not
permanent and may belong to an old network.

Try access in this order:

```bash
ssh harsh@harsh-rpi.local
ssh harsh@192.168.1.216
ssh harsh@192.168.1.68
ssh harsh@192.168.3.2
ssh harsh@10.21.10.17
```

No robot password is stored in Git or the local SSH configuration. Retrieve it
from the private password-manager record. Prefer an SSH key. If no credential
works, use a directly attached keyboard/display and an authorized local account
to run `sudo passwd harsh`, restore `~harsh/.ssh/authorized_keys`, and verify
permissions. If no authorized local recovery account exists, reimage the Pi and
restore from this manual instead of attempting to bypass authentication.

To restore the preferred hostname:

```bash
sudo hostnamectl set-hostname harsh-rpi
sudo apt-get install avahi-daemon
sudo systemctl enable --now avahi-daemon
hostname
```

### User-facing network endpoints

| Interface | Address | Notes |
| --- | --- | --- |
| SSH | `ssh harsh@harsh-rpi.local` | key-based administration preferred |
| Bob web app | `http://harsh-rpi.local:8080` | main phone/computer UI |
| Rosbridge | `ws://harsh-rpi.local:9090` | browser ROS connection |
| Foxglove | `ws://harsh-rpi.local:8765` | diagnostics when launched |
| BLE | advertised as `RobotBLE` | Wi-Fi provisioning and basic mode commands |

Replace the hostname with the current IP if mDNS is unavailable. Ports 8080,
9090, 8765, and BLE have no application-level authentication in this archive.
Use only on a trusted private network or behind a VPN/firewall. Never configure
public router forwarding to them.

## 3. Do this before decommissioning

If the Pi or its storage is still available, complete this section before
packing it away. The archive workstation could not reach the Pi on 2026-09-09.

### 3.1 Stop motion and shut down cleanly

1. Put the robot on blocks so the wheels cannot contact the floor.
2. Use **STOP ROBOT** in the UI or call:

   ```bash
   curl -X POST http://harsh-rpi.local:8080/api/stop
   ```

3. Confirm `/motor_driver_node` is absent or stopped and the wheels are still.
4. Shut down Linux:

   ```bash
   sudo systemctl poweroff
   ```

5. Wait until storage activity has ceased. Open the physical motor cutoff, then
   disconnect the battery and charger.

### 3.2 Record the as-built hardware

- Photograph the complete robot from all sides and every connector before
  unplugging it.
- Label both ends of every cable using the IDs in `WIRING_AND_POWER.md`.
- Record battery model, voltage, connector polarity, charger, fuse, wire gauge,
  regulator/PD board model, L298N terminals/jumpers, motor polarity, and the
  exact physical E-stop path.
- If the separate local CAD archive is available, complete
  `cad/chassis_v2/MEASUREMENTS.md` for actual hardware, even if V3 is ultimately
  used. Otherwise record the same measurements using the wiring and acceptance
  checklists in this documentation set.
- Record the Pi storage device and its serial suffix.
- Store photos and the completed private credential record outside public Git.

### 3.3 Back up Pi-only state

Create an ordinary non-secret data archive:

```bash
mkdir -p /home/harsh/robot_archive
tar -C /home/harsh -czf /home/harsh/robot_archive/bob-local-state.tgz \
  .config/mapping-robot/rooms.json \
  .config/mapping-robot/target_enrollment.json \
  robot_recordings 2>/dev/null
```

Treat this archive as private because maps, room names, recordings, and the
appearance signature can reveal the home and occupants. Copy it to encrypted
storage; do not add it to GitHub.

Back up the assistant key separately through a password manager. Do not put
`assistant.env` in the ordinary archive unless the archive is encrypted.

### 3.4 Preserve the required neural-network model

Production tracking expects:

```text
/home/harsh/ros2_ws/src/depthai-ros/depthai_examples/resources/
mobilenet-ssd_openvino_2021.2_6shave.blob
```

Expected SHA-256:

```text
5150d0e5d18abd0ecb21c8280e09870977358c04a7d2cfa539e1e0f6c2a93e71
```

That `depthai-ros` tree and blob are not in this Git repository. Copy the blob
to the encrypted/private hardware archive if its license permits. Verify it:

```bash
sha256sum /home/harsh/ros2_ws/src/depthai-ros/depthai_examples/resources/mobilenet-ssd_openvino_2021.2_6shave.blob
```

Without this exact file, production person tracking will refuse to start. A
future operator may instead obtain a compatible licensed blob and update both
`blob_path` and `blob_sha256` in `config/person_tracking.yaml`, followed by tests
and a motor-free validation.

### 3.5 Optional full disk image

A compressed, encrypted image of the Pi boot storage is the fastest disaster
recovery method, but it contains credentials and private data. Create it only
with the Pi powered off, using another computer and a verified imaging tool.
Record the image hash, date, source device serial, encryption method, and restore
instructions in the private credential record. Keep at least one copy offline.

## 4. Storage and physical inspection after a long shutdown

Do not connect power immediately.

1. Move the robot to a dry, nonflammable work area with ventilation and a clear
   emergency path.
2. Inspect the battery for swelling, leakage, corrosion, odor, heat, damaged
   insulation, or crushed connectors. Quarantine and replace a suspect pack.
3. Use only the battery manufacturer's compatible charger and storage/recharge
   procedure. Do not revive a deeply discharged or damaged pack by improvising.
4. Inspect printed parts, mast tubes/pins, motor mounts, supports/casters,
   wheels, bumper sliders/springs, fasteners, strain relief, and cable abrasion.
5. Remove dust from the Pi cooler, L298N heatsink, OAK-D lenses, and LD19 window.
6. Confirm OAK-D and LD19 have unobstructed fields of view and no cable carries
   structural load.
7. With outriggers stowed, check four-point ground contact and rocking. The
   robot is tall and narrow; repeat the documented CG/tilt/stability gates.
8. Trace all wiring against `WIRING_AND_POWER.md`. Resolve every discrepancy in
   the code, physical labels, and documentation before power.

## 5. Prepare a replacement Raspberry Pi

The code targets a Raspberry Pi 5 and ROS 2 Jazzy. The old OS image/version was
not live-verified at archive time. A fresh 64-bit Ubuntu 24.04 installation is
the normal ROS Jazzy baseline; if using another image, follow official ROS Jazzy
support requirements and expect package-name differences.

### 5.1 Base operating system

1. Install the 64-bit OS to reliable storage.
2. Create user `harsh` with home `/home/harsh` so service paths remain valid.
3. Set hostname `harsh-rpi` and connect only to the trusted robot network.
4. Enable SSH and install the operator's public key:

   ```bash
   mkdir -p /home/harsh/.ssh
   chmod 700 /home/harsh/.ssh
   # Add only a public key to authorized_keys.
   chmod 600 /home/harsh/.ssh/authorized_keys
   chown -R harsh:harsh /home/harsh/.ssh
   ```

5. Update the OS, reboot, and confirm time, locale, storage, networking, and
   cooling before installing robot software.

### 5.2 Raspberry Pi interfaces

Enable I2C bus 1. GPIO is accessed through `gpiozero`. Verify the platform's
current method for enabling I2C, then check:

```bash
ls -l /dev/i2c-1
sudo i2cdetect -y 1
```

With the IMU connected correctly, address `68` should appear. Do not connect
unverified 5 V signals while doing this.

### 5.3 Install ROS and robot dependencies

Install ROS 2 Jazzy and initialize `rosdep` using the official Jazzy procedure,
then install the dependencies used by this snapshot:

```bash
sudo apt-get update
sudo apt-get install \
  espeak-ng git i2c-tools mpg123 python3-flask python3-gpiozero \
  python3-opencv python3-pyaudio python3-pip python3-smbus \
  python3-venv

python3 -m pip install \
  "depthai>=3.6,<4" openai SpeechRecognition ddgs bless
```

Some modern distributions restrict system-wide pip installs. If so, install
non-ROS Python dependencies in a controlled virtual environment and update the
systemd interpreter paths consistently. Do not let DepthAI 3 leak into the
production DepthAI 2 tracking interpreter.

## 6. Retrieve the exact GitHub snapshot

For a clean recovery using HTTPS:

```bash
git clone --recurse-submodules \
  --branch recommissioning-archive-2026-09-09 \
  https://github.com/harshcshah97-collab/mapping-robot.git \
  /home/harsh/ros2_ws
cd /home/harsh/ros2_ws
git submodule update --init --recursive
git status
```

For a private repository using a configured SSH key:

```bash
git clone --recurse-submodules \
  --branch recommissioning-archive-2026-09-09 \
  git@github.com:harshcshah97-collab/mapping-robot.git \
  /home/harsh/ros2_ws
```

Confirm the checkout is detached at the archive tag, clean, and has both
submodules populated. To resume development:

```bash
cd /home/harsh/ros2_ws
git switch -c recommission/bob-YYYYMMDD
```

Do not initialize Git from `/home/harsh`; the repository root must be
`/home/harsh/ros2_ws`. Before any future push, check `git rev-parse
--show-toplevel` and `git status --short`.

## 7. Restore missing private/runtime artifacts

### 7.1 Production OAK-D model

Restore the exact verified blob to the path in section 3.4. Keep its directory
readable by `harsh`, verify the SHA-256, and do not proceed to tracking if it
differs.

### 7.2 Assistant secret

Retrieve the key from the password manager:

```bash
install -d -m 700 /home/harsh/.config/mapping-robot
install -m 600 /dev/null /home/harsh/.config/mapping-robot/assistant.env
editor /home/harsh/.config/mapping-robot/assistant.env
chown -R harsh:harsh /home/harsh/.config/mapping-robot
```

Enter only:

```dotenv
OPENAI_API_KEY=retrieve-from-password-manager
# Optional model override:
# OPENAI_MODEL=gpt-4o
```

Check permissions without printing the secret:

```bash
stat -c '%U %G %a %n' /home/harsh/.config/mapping-robot/assistant.env
```

### 7.3 Rooms and enrollment

Restore `rooms.json` only if the matching map YAML/PGM pairs are present and
unchanged. If a map was rebuilt or its origin changed, discard/redraw its rooms.
It is usually safer to discard `target_enrollment.json` and enroll the current
subject again in the current clothing and lighting.

### 7.4 Recordings

Restore recordings only if the robot needs them locally. Keep at least 512 MB
free beyond normal OS/build needs; a full filesystem blocks new recording and
can destabilize the system.

## 8. Create the isolated DepthAI 2 environment

Production person tracking requires exactly DepthAI `2.32.0.0`:

```bash
python3 -m venv --system-site-packages /home/harsh/ros2_ws/.venv-depthai2
PYTHONNOUSERSITE=1 /home/harsh/ros2_ws/.venv-depthai2/bin/python3 \
  -m pip install "depthai==2.32.0.0"
PYTHONNOUSERSITE=1 /home/harsh/ros2_ws/.venv-depthai2/bin/python3 \
  -c 'import depthai; print(depthai.__version__)'
```

The final command must print `2.32.0.0`. The normal environment may contain
DepthAI 3 for separate experiments, but production tracking must run through
`.venv-depthai2/bin/python3` with `PYTHONNOUSERSITE=1`.

## 9. Install dependencies and build

```bash
cd /home/harsh/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
./start_build.sh
source install/setup.bash
```

`start_build.sh` applies `patches/ldlidar-pthread.patch` only if the pinned
driver still lacks `<pthread.h>`, then builds packages up to
`my_robot_package` using symlink install.

Run tests:

```bash
cd /home/harsh/ros2_ws/src/my_robot_package
PYTHONPATH="$PWD" pytest -q
cd /home/harsh/ros2_ws
colcon test --packages-select my_robot_package
colcon test-result --verbose
ros2 run my_robot_package calibration_check
```

`calibration_check --strict` is expected to fail until physical calibration
flags are legitimately completed. Never turn flags on only to silence it.

## 10. Install boot services

Copy the archived units:

```bash
sudo cp /home/harsh/ros2_ws/src/my_robot_package/web_ui/robot_web.service \
  /etc/systemd/system/
sudo cp /home/harsh/ros2_ws/src/my_robot_package/web_ui/robot_ble.service \
  /etc/systemd/system/
sudo cp /home/harsh/ros2_ws/src/my_robot_package/web_ui/robot_assistant.service \
  /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot_web.service robot_ble.service robot_assistant.service
```

The web/BLE programs use a small fixed set of root commands for Wi-Fi,
Bluetooth unblock, WLAN power-save, reboot, poweroff, and suspend. Audit the
exact executable paths with `command -v` and create the narrowest possible
`sudoers` policy using `visudo`; do not grant unrestricted passwordless sudo.
The policy is intentionally not guessed or included because command paths and
safe argument matching depend on the restored OS.

Start only the web service first, with motor power disconnected:

```bash
sudo systemctl start robot_web.service
systemctl status robot_web.service --no-pager
journalctl -u robot_web.service -n 100 --no-pager
```

It should start Flask on 8080 and rosbridge on 9090 while leaving the drive
stack stopped. Then start and inspect BLE. Start the assistant only after audio,
network, API key, billing limits, and the web service have been verified.

## 11. Software-only and unpowered checks

Keep the battery/motor supply disconnected.

```bash
source /opt/ros/jazzy/setup.bash
source /home/harsh/ros2_ws/install/setup.bash
ros2 pkg executables my_robot_package
ros2 launch my_robot_package manual_control.launch.py --show-args
ros2 launch my_robot_package bringup_and_map.launch.py --show-args
ros2 launch my_robot_package navigation.launch.py --show-args
ros2 launch my_robot_package person_tracking.launch.py --show-args
```

Check hardware independently:

```bash
i2cget -y 1 0x68 0x75
ls -l /dev/ttyUSB0
lsusb
arecord -l
vcgencmd get_throttled
```

Expected IMU identity is `0x68`. The LD19 is expected at `/dev/ttyUSB0` and
230400 baud. If device enumeration differs, create/update a stable udev rule
before relying on it.

Test each bumper and encoder input without motor power. Confirm no two processes
own BCM9/11/10. A bumper must publish `/safety/stop: true` while held and false
after reliable mechanical release.

## 12. First powered test, wheels raised

Have a second person at the physical cutoff. Keep all autonomous launch files
stopped. Connect only the verified power path.

1. Start the web service and open the UI.
2. Select Drive. Confirm these nodes:

   ```bash
   ros2 node list
   # Expected project nodes: motor_driver_node, encoder_odom_node, bumper_node
   # plus twist_mux and rosbridge.
   ```

3. Hold forward for less than one second at the lowest practical request.
   Confirm both wheels rotate forward and encoder signs produce positive forward
   odometry. Release; motion must stop immediately.
4. Stop publishing `/cmd_vel`. The 0.6-second motor watchdog must stop the wheels.
5. Press each bumper while commanding a wheel. `/safety/stop` must stop both.
6. Use **STOP ROBOT**. Confirm the mode process exits and GPIO is released.
7. Test the physical motor cutoff. It must remove motor power regardless of ROS.
8. Repeat reverse and left/right turns briefly. If any direction or encoder sign
   is wrong, disconnect power and correct labels/config before proceeding.

Do not enable closed-loop PI. The archive defaults it off because geometry and
wheel speed are not physically confirmed.

## 13. Sensor and transform validation

With wheels raised or motor power disconnected:

```bash
ros2 launch my_robot_package bringup_and_map.launch.py
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /imu/data
ros2 topic hz /ultrasonic_distance
ros2 topic hz /ir/left
ros2 topic hz /ir/right
ros2 run tf2_tools view_frames
```

Verify:

- LD19 orientation: forward obstacles appear at the expected scan angle.
- IMU: axes match the URDF; gyro is near zero while still; yaw sign is correct.
- Encoder distance and turn angle match tape/angle measurements.
- Ultrasonic/IR fields point where their URDF frames claim.
- Bumper cloud points appear on the contacted side.
- TF chain is `map -> odom -> base_footprint -> base_link -> sensors` when a
  mapping/localization owner exists.

The current URDF sensor transforms, 0.060 m wheel diameter, 0.240 m separation,
and Nav2 footprint are provisional. Measure the finished robot and update:

- `config/hardware_calibration.yaml`
- `config/calibration_status.yaml`
- `config/nav2_params.yaml`
- `urdf/my_robot.urdf`
- bumper contact coordinates in `bumper_node.py`

Rebuild and retest after every change.

## 14. Recalibration gates

Complete the README calibration procedure and set a status flag true only when
evidence exists for it:

- `wheel_geometry_measured`
- `wheel_speed_measured`
- `closed_loop_tuned_wheels_raised`
- `closed_loop_tuned_on_floor`
- `robot_footprint_measured`
- `oakd_mount_transform_measured`
- `oakd_depth_benchmark_passed`
- `imu_mount_and_bias_checked`

Before floor motion also repeat the archived stability gates: loaded CG,
support-edge margin, four-contact rocking, mast proof load/joint retention, head
deflection, static tilt in four directions, progressive restrained
braking/turning, bumper return, and hardware cutoff.

The local-only V3 CAD is a provisional fit-check design. Only its fit coupon was
released for printing at archive time. Do not treat any locally retained
structural STL/3MF files as a manufacturing release.

## 15. Restore and verify the UI

With `robot_web.service` active:

```bash
hostname -I
curl http://127.0.0.1:8080/api/status
ss -lntp | grep -E ':(8080|9090)\b'
```

From a phone/computer on the same trusted network, open:

```text
http://harsh-rpi.local:8080
```

If mDNS fails, use the IPv4 address reported by `hostname -I`, the router's DHCP
lease list, or BLE's IP characteristic.

Verify the app while motors are disconnected:

- Header shows current mode, Pi temperature, and throttle state.
- ROS connects to port 9090.
- Maps list only complete YAML/image pairs.
- STOP is visible from every screen.
- Drive buttons publish only while held and publish zero on release/hidden tab.
- Vision controls show tracking status/events.
- Bob text requests report a useful service/offline error if assistant is down.
- Wi-Fi scan/configuration and reboot/power controls are available only after
  the narrow sudo policy is intentionally installed.

The app may require internet for pinned CDN-hosted ROSLib/ROS2D assets. Full
offline operation requires vendoring those browser dependencies and updating
`index.html`/service-worker caching.

## 16. Verify each operating mode

Use one mode at a time through the UI. Keep a spotter and clear space for every
first floor run.

### 16.1 Drive

Already validate it with wheels raised. On the floor, begin with short commands
at low speed and verify stop distance, straight tracking, turning, bumper stop,
watchdog, UI STOP, Wi-Fi loss behavior, and physical cutoff.

### 16.2 Mapping

Normal mapping:

```bash
cd /home/harsh/ros2_ws
./start_mapping.sh
```

or use the app Map screen. Confirm fresh scan/odom/IMU/range data and Foxglove
before allowing the wall follower to move. Legacy wall following owns bumper
GPIO directly and can reverse after contact.

Save maps from the app or:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/harsh/ros2_ws/src/my_robot_package/maps/NEW_MAP_NAME
```

Commit both YAML and image only after verifying the map. Frontier exploration
is optional and remains blocked until wheel geometry, wheel speed, footprint,
and IMU gates are true.

### 16.3 Navigation

```bash
ros2 launch my_robot_package navigation.launch.py \
  map:=/home/harsh/ros2_ws/src/my_robot_package/maps/my_house_map_0515.yaml
```

Set the initial pose before sending a goal. Confirm AMCL localization and
costmaps, then send a nearby supervised goal. Verify LiDAR/range/bumper layers,
recovery behavior, STOP, and localization after turns. Named rooms are tied to
one map; redraw them after any map origin/geometry change.

### 16.4 OAK-D motor-free validation

First verify the camera without motion. Production tracking checks the exact
DepthAI version and blob checksum. Confirm preview and status, then enroll with
exactly one person visible. Test photo/recording and stop recording cleanly.

For optional depth costmaps, measure the OAK-D transform, update the URDF and
gates, then benchmark:

```bash
# Terminal 1
ros2 launch my_robot_package oakd_depth_benchmark.launch.py

# Terminal 2
ros2 run my_robot_package perception_benchmark \
  --duration 120 --minimum-cloud-hz 5 --maximum-temperature-c 75
```

Only a passing result permits enabling OAK-D points in Nav2.

### 16.5 Person following and framing

```bash
ros2 launch my_robot_package person_tracking.launch.py
```

The launch starts IDLE. Confirm fresh LD19, ultrasonic, both IR, and bumper data.
Repair/calibrate the known bad left FC-51 and restore
`ir_stop_requires_both: false`; the archived degraded setting can miss an
obstacle seen by only one IR sensor.

Stand alone, fully visible, near the intended distance, enroll, then test
`keep_frame` before slow following. The target is a clothing-color signature,
not verified identity. Re-enroll after clothing/lighting changes. Confirm stop
on target loss, every stale sensor, obstacle thresholds, bumper, excessive Pi/OAK
temperature, undervoltage/throttling, watchdog, UI STOP, and physical cutoff.

The archived apartment profile uses a 0.60 m target distance, 0.10 m/s maximum
follow speed, 0.30 m LiDAR stop, and 0.20 m ultrasonic/IR stop. Revalidate these
against the completed robot's measured footprint and stopping distance.

## 17. Restore and verify Bob assistant

Prerequisites: robot web service active on loopback 8080, trusted internet,
valid API key with billing/usage limits, working microphone/speaker, and Vision
running for camera questions.

Start foreground first:

```bash
set -a
source /home/harsh/.config/mapping-robot/assistant.env
set +a
/home/harsh/ros2_ws/start_assistant.sh
```

Check text without relying on the microphone:

```bash
ros2 topic pub --once /assistant/text_query std_msgs/msg/String \
  '{data: "{\"id\":\"manual-test\",\"text\":\"Say system ready in five words\"}"}'
ros2 topic echo /assistant/text_response
```

Then test wake phrase, speech input, short spoken output, web search, mode switch,
named-room navigation, and current camera scene. Robot tool calls go through the
local Flask mode manager. `allow_shell_commands` must remain false for normal
operation.

Enable the service only after foreground success:

```bash
sudo systemctl enable --now robot_assistant.service
systemctl status robot_assistant.service --no-pager
journalctl -u robot_assistant.service -n 100 --no-pager
```

If audio works interactively but not in systemd, verify the `harsh` user session,
`/run/user/UID`, PipeWire/Pulse socket, device permissions, and the runtime
environment set by `start_assistant.sh`.

## 18. BLE and phone access

Start and inspect BLE:

```bash
sudo systemctl enable --now bluetooth.service robot_ble.service
bluetoothctl show
journalctl -u robot_ble.service -n 100 --no-pager
```

Scan for `RobotBLE`. It can return the WLAN IP, scan/configure Wi-Fi, and forward
allowlisted start/stop commands to the local web app. Treat BLE setup as local
provisioning, not secure authorization. Disable the service when not needed if
the robot is used around untrusted people.

The Swift files in `web_ui/` are source fragments, not a complete archived iOS
application. The browser app at port 8080 is the complete UI in this repository.

## 19. Acceptance checklist

Do not declare the robot recommissioned until every applicable item is true.

### Access and software

- [ ] Git tag cloned and both submodules initialized.
- [ ] Working branch created; repository root is `/home/harsh/ros2_ws`.
- [ ] Tests pass and build/test results are archived.
- [ ] SSH key and fallback local recovery work.
- [ ] `harsh-rpi.local` and current DHCP address are documented privately.
- [ ] UI 8080, rosbridge 9090, and optional Foxglove 8765 are private-network only.
- [ ] API key permissions are 0600 and the value is absent from Git/logs.
- [ ] Required MobileNet blob exists and matches its SHA-256.

### Electrical and mechanical

- [ ] Battery, charger, fuse, polarity, wire gauge, regulator, and L298N traced.
- [ ] Physical motor cutoff/E-stop interrupts motor energy independently.
- [ ] HC-SR04 ECHO, IR outputs, and encoder outputs are 3.3 V safe.
- [ ] Grounds/reference and high-current routing match the wiring guide.
- [ ] Every connector labeled and strain-relieved; optics/cooling unobstructed.
- [ ] Bumpers return freely; no duplicate GPIO owners.
- [ ] Loaded CG, support, tilt, mast, braking, and turning gates pass.

### Runtime behavior

- [ ] Motor direction, encoder signs, watchdog, bumpers, UI STOP, and cutoff pass.
- [ ] Scan, odom, IMU, range, bumper, TF, and thermal data are fresh/correct.
- [ ] Measured wheel geometry, footprint, transforms, and IMU offsets are recorded.
- [ ] Mapping saves a valid paired map.
- [ ] Navigation localizes and reaches a short supervised goal safely.
- [ ] OAK-D motor-free preview, enrollment, photo, recording, and thermal test pass.
- [ ] Left FC-51 defect is repaired and degraded both-IR behavior removed.
- [ ] Tracking stops on every target/sensor/obstacle/thermal/power safety gate.
- [ ] Bob text, microphone, speaker, app tools, and no-shell policy pass.
- [ ] Service restart and full reboot return to stopped, UI-available state.

## 20. Troubleshooting map

| Symptom | First checks |
| --- | --- |
| SSH name fails | use current IP, router DHCP list, BLE IP, `avahi-daemon`, same LAN |
| Git clone lacks packages | `git submodule update --init --recursive` |
| Build fails in LD19 logger | run `./start_build.sh`; confirm pinned submodule and patch |
| `/dev/ttyUSB0` absent | USB power/cable, `dmesg`, permissions, changed device name/udev |
| IMU absent | I2C enabled, 3.3 V/GND/SDA/SCL, bus 1, address 0x68 |
| UI unavailable | `robot_web` status/log, port 8080, firewall, hostname/IP |
| UI loads but ROS is disconnected | rosbridge child/log, port 9090, browser console, CDN access |
| Buttons say started but no motion | enrollment/mode, mux topics, safety Bool, watchdog, calibration, motor cutoff |
| Motor will not stop | use hardware cutoff immediately; do not resume until repaired |
| Tracker refuses startup | exact DepthAI 2.32 interpreter, blob path/hash, USB, OAK temperature |
| Tracker remains blocked | inspect `/tracking/status`; check all required sensor timestamps and enrollment |
| Assistant has no answer | API key/network/billing/model, assistant log, text topics, web service loopback |
| Assistant hears but cannot speak | EMEET output, PipeWire/Pulse user socket, `mpg123`, systemd user runtime |
| Navigation has TF errors | one owner each for map->odom and odom->base; URDF/state publisher; clock |
| Maps absent in UI | YAML must reference an existing image in package maps directory |
| Named room is wrong | room belongs to old map/origin; delete and redraw after localization |
| Pi reports throttling/undervoltage | stop motors, inspect supply/cables/load/cooling; do not use hot override |

Useful inspection commands:

```bash
ros2 node list
ros2 topic list -t
ros2 node info /motor_driver_node
ros2 topic echo /safety/stop
ros2 topic echo /tracking/status
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_tools view_frames
systemctl status robot_web robot_ble robot_assistant --no-pager
journalctl -b -p warning --no-pager
vcgencmd measure_temp
vcgencmd get_throttled
```

## 21. Normal shutdown and re-archive

1. Stop recording and save any active map.
2. Use UI STOP and verify the base is still.
3. Shut down Linux cleanly.
4. Wait for storage activity to cease.
5. Open motor cutoff, disconnect battery, then disconnect charger/peripherals.
6. Update measurement/wiring/access records with any changes.
7. Back up new maps, rooms, recordings, and required private artifacts.
8. Run tests, commit source/docs, create a new dated immutable tag, and push the
   branch and tag.
9. From a second machine, clone the tag with submodules and verify the archive
   before relying on it.

## 22. Maintenance rule

Whenever hardware, wiring, a topic, a launch mode, a secret location, a service,
or a user-facing endpoint changes, update the corresponding source file and this
documentation in the same commit. A future operator should never have to choose
between trusting the robot and trusting the manual.
