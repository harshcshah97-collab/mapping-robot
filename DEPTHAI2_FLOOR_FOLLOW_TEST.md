# DepthAI 2.32 bounded floor-follow test

[Start here](docs/START_HERE.md) |
[Recommissioning manual](docs/RECOMMISSIONING_MANUAL.md) |
[Wiring](docs/WIRING_AND_POWER.md) |
[Nodes and data flow](docs/NODES_AND_DATA_FLOW.md)

This is a one-off diagnostic mode for the OAK-D Lite regression in DepthAI 3.
It does **not** replace the normal tracking stack and it does not downgrade the
normal Python environment.

The test is deliberately narrow:

- exactly one visible person with valid OAK stereo depth;
- LiDAR, bumper heartbeat, encoder odometry, Pi power state, and temperatures
  must all be healthy before arming;
- private command topic with one motor driver and no twist mux, navigation,
  mapping, Bob, or normal person follower;
- at most 30 seconds, 0.05 m/s forward, 0.15 rad/s turn, 1.50 m translation,
  and 0.35 rad rotation;
- no reverse, no reconnect, and no automatic re-arm;
- any fault publishes zero, ends the controller, and shuts down the launch.

## Pi preparation

Create an isolated environment once. Do not install DepthAI 2 into the normal
user or ROS Python environment.

```bash
python3 -m venv --system-site-packages /home/harsh/ros2_ws/.venv-depthai2
PYTHONNOUSERSITE=1 /home/harsh/ros2_ws/.venv-depthai2/bin/python3 \
  -m pip install --no-cache-dir depthai==2.32.0.0
```

Build the package after sourcing ROS:

```bash
cd /home/harsh/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_robot_package --symlink-install
source install/setup.bash
```

## Test sequence

1. Open the enclosure for ventilation. Let the Pi cool below 70 C.
2. Put the robot on flat ground, at least 2 m from stairs, pets, feet, and
   obstacles. Keep a hand on the physical motor-power switch.
3. Keep the LiDAR connected and verify its zero-degree direction faces forward.
4. Stand alone, fully visible, centered, about 2 m in front of the OAK-D.
5. Start the launch. It remains disarmed:

   ```bash
   ros2 launch my_robot_package depthai2_floor_follow_test.launch.py \
     enable_motion:=true
   ```

6. Confirm `/depthai2_test/status` reports `"ready": true`, then arm once:

   ```bash
   ros2 service call /depthai2_test/arm std_srvs/srv/Trigger '{}'
   ```

The launch exits after the 30-second session, the first safety fault, or the
40-second whole-process timeout. Do not click dashboard motion controls while
this test is running.

The motor geometry is still uncalibrated, so the reported speeds and odometry
are only guard estimates. A successful short test proves the v2 camera/control
path; it does not certify the robot for unattended following. The experimental
forward threshold is 0.60 m, equal to the independent LiDAR stop distance, so
the robot should stop before entering that range and never reverses.
