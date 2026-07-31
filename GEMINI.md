# Companion Robot Project

## 1. Project Overview

- **Goal:** To create a companion robot.
- **Hardware:**
    - **Sensors:**
        - Lidar
        - OAK-D Lite camera
        - 2x IR sensors
        - 1x Ultrasonic sensor
        - IMU
    - **Actuators:**
        - 2x Encoder motors
    - **Audio:**
        - Speaker
        - Microphone

## 2. Workspace Structure

This workspace contains the following packages:

### `depthai-ros`

This is a meta-package containing packages for Luxonis OAK-D devices.

- **`depthai_bridge`**: Bridge between `depthai` library and ROS messages.
- **`depthai_descriptions`**: URDF models for DepthAI cameras.
- **`depthai_examples`**: Example usage of `depthai-ros` packages.
- **`depthai_filters`**: ROS nodes for filtering and processing data from DepthAI cameras.
- **`depthai_ros_driver`**: The main driver node for DepthAI cameras.
- **`depthai_ros_msgs`**: Custom ROS messages for `depthai-ros`.

### `ldlidar_stl_ros2`

- **Description:** ROS2 package for LD-LIDAR sensors (LD06, LD19, STL27L).
- **Dependencies:** rclcpp, sensor_msgs, ros2launch

### `m-explore-ros2`

This is a meta-package for autonomous exploration.

- **`explore_lite`**: A lightweight frontier-based exploration algorithm.
- **`explore_lite_msgs`**: Custom messages for `explore_lite`.
- **`multirobot_map_merge`**: A package for merging maps from multiple robots.

### `my_robot_package`

This package contains the custom nodes and logic for the companion robot.

- **Description:** TODO: Package description
- **Dependencies:** (No dependencies listed in package.xml)

## 3. Python Nodes

The following Python nodes were found in the workspace:

### `my_robot_package`

- `assistant_node.py`: (Likely handles voice interaction and assistant features)
- `person_follower_node.py`: (Likely for following a person using camera data)
- `bumper_node.py`: (Likely reads data from bumper/IR sensors)
- `encoder_odom_node.py`: (Likely publishes odometry from motor encoders)
- `imu_node.py`: (Likely publishes IMU data)
- `ir_driver.py`: (Driver for IR sensors)
- `motor_driver_node.py`: (Controls the motors)
- `oakd_tracker_node.py`: (Likely uses the OAK-D camera for object tracking)
- `random_walk.py`: (A simple autonomous movement behavior)
- `ultra_sensor_node.py`: (Likely reads data from the ultrasonic sensor)
- `wall_follower.py`: (A behavior for following walls)
- `check_vision.py`: (A test or utility script for vision)
- `oakd_ai_test.py`: (A test script for OAK-D AI features)

#### `web_ui`
- `ble_wifi_scanner.py`: (Scans for Bluetooth and WiFi networks)
- `robot_web_server.py`: (Hosts a web interface for the robot)

### `depthai-ros`
- `depthai_examples/scripts/markerPublisher.py`: Publishes markers for visualization.
- `depthai_ros_driver/scripts/obj_pub.py`: Publishes object detection data.

## 4. Launch Files

The following launch files are available in the workspace:

### `my_robot_package`
- `bringup_and_map.launch.py`: Launches the robot's main nodes for mapping.
- `ld19.launch.py`: Launches the LD19 lidar driver.
- `navigation.launch.py`: Launches the navigation stack.
- `person_tracking.launch.py`: Launches nodes for person tracking.

### `depthai-ros`
(Contains many example launch files for various camera functionalities, including stereo vision, object tracking, and point cloud generation.)

- `depthai_examples/launch/`: Contains various examples like `rgb_stereo_node.launch.py`, `tracker_yolov4_node.launch.py`, etc.
- `depthai_filters/launch/`: Contains examples for filters like `example_wls_filter.launch.py`.
- `depthai_ros_driver/launch/`: Contains main launch files like `camera.launch.py`, `pointcloud.launch.py`, and `rtabmap.launch.py`.

### `ldlidar_stl_ros2`
- `ld06.launch.py`: Launches the LD06 lidar.
- `ld19.launch.py`: Launches the LD19 lidar.
- `stl27l.launch.py`: Launches the STL27L lidar.
- `viewer_ld06.launch.py`: Launches the LD06 lidar with a viewer.
- `viewer_ld19.launch.py`: Launches the LD19 lidar with a viewer.
- `viewer_stl27l.launch.py`: Launches the STL27L lidar with a viewer.

### `m-explore-ros2`
- `explore/launch/explore.launch.py`: Launches the exploration node.
- `map_merge/launch/map_merge.launch.py`: Launches the map merging node.

## 5. Build Instructions

To build the workspace, run the following commands from the `ros2_ws` directory:

```bash
colcon build
```

To source the workspace, run:
```bash
source install/setup.bash
```
