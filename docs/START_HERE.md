# Start Here: Bob Robot Repository Guide

[Recommissioning manual](RECOMMISSIONING_MANUAL.md) |
[Wiring](WIRING_AND_POWER.md) |
[Nodes and data flow](NODES_AND_DATA_FLOW.md) |
[File guide](FILE_GUIDE.md)

Use this page as the front door to the project. It explains how GitHub is
organized, how to move between folders, where the robot code is stored, and
which document to open for a particular task.

## Quick start

If you only remember one route, use this:

1. Open the [mapping-robot repository](https://github.com/harshcshah97-collab/mapping-robot).
2. Open the branch selector above the file list.
3. Select `codex/companion-robot-hardening`.
4. Open `docs`.
5. Open `START_HERE.md`.

You can also bookmark the [Start Here page on the working branch](https://github.com/harshcshah97-collab/mapping-robot/blob/codex/companion-robot-hardening/docs/START_HERE.md).

## Choose what you need

| I want to... | Open this |
| --- | --- |
| Understand GitHub or find a folder | this `START_HERE.md` page |
| Rebuild and recommission Bob | [Recommissioning manual](RECOMMISSIONING_MANUAL.md) |
| Find a wire, GPIO pin, USB connection, or circuit | [Wiring and power](WIRING_AND_POWER.md) |
| Understand ROS nodes, topics, modes, ports, UI, BLE, or Bob assistant | [Nodes and data flow](NODES_AND_DATA_FLOW.md) |
| Learn what a source file does | [Repository and file guide](FILE_GUIDE.md) |
| Record private access information safely | [Credential template](CREDENTIALS.template.md) |
| Get a shorter technical overview | [Main README](../README.md) |
| Run the bounded DepthAI floor test | [DepthAI floor-test procedure](../DEPTHAI2_FLOOR_FOLLOW_TEST.md) |

## The four GitHub ideas you need

### Repository

A repository is the whole project folder plus its change history. This
repository is named `mapping-robot`.

It contains the robot software, documentation, configuration, maps, tests, and
references to two external ROS packages. CAD and secrets are deliberately not
stored on GitHub.

### Branch

A branch is a named working version of the repository. New commits can be added
to it, so its contents can change over time.

The current organized working branch is:

```text
codex/companion-robot-hardening
```

When following links between files, stay on this branch unless you deliberately
want the frozen archive.

### Commit

A commit is one recorded change to the project. It has a unique identifier such
as `314a20e`. A branch moves forward as new commits are added.

Click the commit message above the GitHub file list to see what changed in the
most recent commit. Click **History** while viewing a file to see that file's
earlier changes.

### Tag

A tag is a name attached to one exact commit. It is normally used as a fixed
bookmark for a release or archive.

The original frozen archive is:

```text
recommissioning-archive-2026-09-09
```

The tag should remain unchanged while the working branch receives documentation
improvements. Therefore, the branch may eventually be easier to read than the
tag even though the underlying archived robot code began at the same point.

## What you see on a repository page

| GitHub item | What it means | How to use it |
| --- | --- | --- |
| Repository name | `mapping-robot`, the project root | click it in the path to return to the top |
| Branch selector | selected branch or tag | choose `codex/companion-robot-hardening` |
| File list | folders first, then files | click a name to open it |
| Breadcrumb path | such as `mapping-robot / src / my_robot_package` | click an earlier name to move back up |
| Rendered README | instructions for the current folder | check the path to know which README it is |
| **Code** button | clone and download options | use Git clone for a working copy with submodules |
| **History** | commits affecting the current folder or file | inspect previous versions |
| Search field | repository-wide code and file search | search for a node, topic, filename, or GPIO |

GitHub links normally contain either `/tree/` or `/blob/`:

- `/tree/BRANCH/path` means a folder.
- `/blob/BRANCH/path/file` means a file.

## How to move between folders

Suppose you are viewing:

```text
mapping-robot / src / my_robot_package / launch / navigation.launch.py
```

- Click `launch` to see all launch files.
- Click `my_robot_package` to see the whole project package.
- Click `src` to see all ROS source packages.
- Click `mapping-robot` to return to the repository root.
- Use the browser Back button to return to the page you just left.

The breadcrumb is the safest way to navigate. It also tells you whether two
files with the same name are actually in different folders.

## Why README appears in many places

`README.md` is an ordinary Markdown instruction file. GitHub automatically
shows the README belonging to the folder you are currently viewing.

| README | Scope |
| --- | --- |
| `/README.md` | overview of the complete Bob project |
| `/docs/START_HERE.md` | beginner navigation hub |
| `/docs/RECOMMISSIONING_MANUAL.md` | complete recovery procedure |
| `/src/ldlidar_stl_ros2/README.md` | upstream LD19 driver documentation |
| `/src/m-explore-ros2/README.md` | upstream exploration documentation |

Always check the breadcrumb path. The word `README` alone does not identify
which part of the project it describes.

## Repository map

```text
mapping-robot/
|-- README.md                       Short project overview
|-- DEPTHAI2_FLOOR_FOLLOW_TEST.md   Special bounded hardware-test procedure
|-- docs/                           Human documentation and recovery guides
|-- patches/                        Tracked fix for the pinned LiDAR source
|-- src/                            All ROS packages and Bob application code
|   |-- my_robot_package/           Code owned by this robot project
|   |-- ldlidar_stl_ros2/           External pinned LiDAR-driver submodule
|   `-- m-explore-ros2/             External pinned exploration submodule
|-- start_build.sh                  Build the ROS workspace
|-- start_mapping.sh                Start normal mapping
|-- start_assistant.sh              Start Bob's assistant in the foreground
`-- .gitmodules                     External submodule URLs
```

The local computer also has a `cad/` folder. It is ignored by Git and is not
available in the GitHub repository, branch, tag, or downloaded ZIP.

## Inside `docs`

Path: `mapping-robot / docs`

| File | Contents |
| --- | --- |
| `START_HERE.md` | this navigation guide and documentation index |
| `RECOMMISSIONING_MANUAL.md` | complete recovery, access, installation, calibration, UI, assistant, tests, and acceptance |
| `WIRING_AND_POWER.md` | GPIO defaults, physical pins, signal circuit, provisional power topology, and first power |
| `NODES_AND_DATA_FLOW.md` | ROS nodes, topics, transforms, modes, web server, BLE, assistant, and ports |
| `FILE_GUIDE.md` | detailed role of every project-owned file and common edit workflows |
| `CREDENTIALS.template.md` | blank private-record fields, never actual passwords or keys |

Every major document has a navigation row at the top. Click **Start here** from
any of them to return to this page.

## Inside `src`

Path: `mapping-robot / src`

This folder holds the ROS source packages.

### `src/my_robot_package`

This is Bob's project-owned application package. Most work happens here.

```text
src/my_robot_package/
|-- my_robot_package/    Python node and algorithm source code
|-- launch/              Groups of nodes started for each operating mode
|-- config/              Tunable YAML parameters
|-- web_ui/              Browser app, Flask server, BLE, and systemd units
|-- maps/                Saved map YAML/image pairs
|-- urdf/                Robot frames and sensor transforms
|-- test/                Automated functional and contract tests
|-- setup.py             Python executables and installed data files
|-- package.xml          ROS dependencies and package metadata
`-- setup.cfg            ROS Python executable installation settings
```

### `src/ldlidar_stl_ros2`

This external Git submodule contains the LD19 LiDAR driver. The main repository
records the exact upstream commit. GitHub may show a submodule with an
arrow/commit indicator, and clicking it may open the external repository. Use
the browser Back button or click the main `mapping-robot` link to return.

### `src/m-explore-ros2`

This external Git submodule provides `explore_lite` and map merge packages. Bob
uses `explore_lite` only for optional calibration-gated frontier exploration.

## Inside the Python source folder

Path: `mapping-robot / src / my_robot_package / my_robot_package`

| File | Main responsibility |
| --- | --- |
| `motor_driver_node.py` | sends GPIO/PWM commands to both motors |
| `encoder_odom_node.py` | converts wheel encoder ticks into odometry |
| `imu_node.py` | reads the MPU6050 over I2C |
| `ultra_sensor_node.py` | publishes HC-SR04 distance readings |
| `ir_driver.py` | publishes left and right IR obstacle states |
| `bumper_node.py` | publishes contact obstacles and immediate safety stop |
| `wall_follower.py` | legacy autonomous wall-follow mapping behavior |
| `person_follower_node.py` | OAK-D detection, enrollment, follow, media, and safety |
| `assistant_node.py` | Bob voice/text assistant, camera questions, speech, and tools |
| `calibration_check.py` | checks calibration consistency |
| `perception_benchmark.py` | measures camera rate and Pi temperature |
| `*_logic.py` | hardware-independent calculations covered by tests |

Use [Nodes and data flow](NODES_AND_DATA_FLOW.md) for publishers, subscribers,
topics, and diagrams. Use [File guide](FILE_GUIDE.md) for the full file list.

## Inside `launch`

Path: `mapping-robot / src / my_robot_package / launch`

A launch file starts a coordinated set of ROS nodes.

| Launch file | What it starts |
| --- | --- |
| `manual_control.launch.py` | minimal manual Drive stack |
| `bringup_and_map.launch.py` | LiDAR, sensors, SLAM, and mapping behavior |
| `navigation.launch.py` | saved-map localization and Nav2 |
| `person_tracking.launch.py` | production OAK-D person tracking |
| `ld19.launch.py` | LD19 driver only |
| `oakd_depth_benchmark.launch.py` | motor-free OAK-D point-cloud benchmark |
| `depthai2_floor_follow_test.launch.py` | bounded test, motion disabled by default |

Do not start multiple motion launch files together. The web app is designed to
own one mode and stop it before starting another.

## Inside `config`

Path: `mapping-robot / src / my_robot_package / config`

| File | What it controls |
| --- | --- |
| `hardware_calibration.yaml` | GPIO, wheel geometry, motors, IMU, OAK-D gates |
| `calibration_status.yaml` | completed physical calibration evidence |
| `person_tracking.yaml` | tracking speed, distance, safety, thermal, and media |
| `twist_mux.yaml` | command priority for all motion modes |
| `nav2_params.yaml` | localization, planning, control, costmaps, and footprint |
| `ekf.yaml` | encoder/IMU fusion |
| `mapper_params_online_async.yaml` | SLAM Toolbox mapping behavior |
| `explore.yaml` | frontier-exploration behavior |
| `depthai2_floor_follow_test.yaml` | bounded test limits only |

Do not mark calibration flags true merely to clear an error. They represent
physical safety evidence.

## Inside `web_ui`

Path: `mapping-robot / src / my_robot_package / web_ui`

| File | What it does |
| --- | --- |
| `index.html` | app screens and controls |
| `style.css` | responsive visual layout |
| `main.js` | buttons, ROS, maps, teleop, tracking, rooms, and assistant |
| `robot_web_server.py` | web API and exclusive ROS mode manager |
| `semantic_rooms.py` | validates and stores named-room goals |
| `ble_wifi_scanner.py` | BLE Wi-Fi provisioning and basic commands |
| `robot_web.service` | starts web app and rosbridge at boot |
| `robot_ble.service` | starts BLE provisioning at boot |
| `robot_assistant.service` | starts Bob's assistant at boot |
| PWA files | installable web-app manifest, cache, and icon |
| Swift files | iOS wrapper references, not a complete Xcode app |

The finished browser interface is reached on the robot at:

```text
http://harsh-rpi.local:8080
```

## Inside `maps`, `urdf`, and `test`

### `maps`

Saved maps normally come as a `.yaml` metadata file plus a `.pgm` image. The app
offers only map YAML files whose referenced images also exist.

### `urdf`

`my_robot.urdf` describes the transform relationships between the base and
sensor frames. Several archived measurements are provisional.

### `test`

Tests cover motor/tracking logic, calibration contracts, production DepthAI
requirements, the web app, rooms, and the bounded floor test. Tests cannot prove
electrical voltage, motor direction, stopping distance, or physical stability.

## Find information without browsing every folder

Use GitHub's repository search for a filename, topic, node, or setting, such as:

```text
person_follower_node.py
/tracking/status
motor_driver_node
left_in1_gpio
target_distance_m
robot_web.service
```

When a result opens, check both the selected branch and breadcrumb. Results from
another branch may show older code.

## Reading a file on GitHub

- Click a line number to create a link to that exact line.
- Use **Raw** for the original file text without GitHub formatting.
- Use **History** to see previous versions.
- Use **Blame** to see which commit last changed each line.
- Click the pencil only when you intend to edit and create a commit.

For ordinary reading, you do not need to download or edit anything.

## Branch versus frozen tag in daily use

Use the working branch for the clearest documentation, future fixes, and current
development. Use the dated tag for the exact archived baseline.

To switch, open the branch selector. Choose a name under **Branches** for a
branch or under **Tags** for the frozen snapshot.

## Downloading versus cloning

**Download ZIP** is suitable for reading ordinary files, but it is not the
preferred robot recovery method because Git submodules may not be included as
complete source checkouts.

Clone the frozen archive with submodules:

```bash
git clone --recurse-submodules \
  --branch recommissioning-archive-2026-09-09 \
  https://github.com/harshcshah97-collab/mapping-robot.git \
  /home/harsh/ros2_ws
```

Clone the newest organized branch:

```bash
git clone --recurse-submodules \
  --branch codex/companion-robot-hardening \
  https://github.com/harshcshah97-collab/mapping-robot.git \
  mapping-robot
```

## Information intentionally not on GitHub

- Linux and Wi-Fi passwords.
- OpenAI API keys, tokens, and SSH private keys.
- Room/enrollment state and private recordings.
- Pi disk images.
- The production MobileNet blob unless separately archived under its license.
- The complete `cad/` directory.

Use the credential template to record where these items are stored without
putting their values in GitHub.

## Recommended recommissioning reading order

1. Read this page to understand where everything is.
2. Read the complete [recommissioning manual](RECOMMISSIONING_MANUAL.md) before
   touching the stored robot.
3. Verify connections with [wiring and power](WIRING_AND_POWER.md).
4. Use [nodes and data flow](NODES_AND_DATA_FLOW.md) while starting ROS modes.
5. Use the [file guide](FILE_GUIDE.md) when troubleshooting or changing code.

From any document, click **Start here** in its top navigation row to return to
this index.
