#!/usr/bin/env python3
"""One-shot, guarded bounded floor-follow test using DepthAI 2.32.

This is intentionally isolated from the normal DepthAI 3 tracking node.  It
starts disarmed, has no reconnect loop, publishes on a private motor topic, and
terminates the whole test after completion or the first latched fault.
"""

import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

import depthai as dai
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from my_robot_package.depthai2_floor_follow_logic import (
    FollowLimits,
    bounded_follow_command,
    current_throttle_bits,
    normalize_angle,
    planar_distance,
)


class DepthAI2FloorFollowTestNode(Node):
    """Drive one small bounded follow session after explicit arming."""

    PERSON_LABEL = 15
    REQUIRED_DEPTHAI_VERSION = "2.32.0.0"

    def __init__(self):
        super().__init__("depthai2_floor_follow_test")
        self._declare_parameters()
        self._load_parameters()
        self._validate_runtime()

        self.command_publisher = self.create_publisher(
            Twist, "/depthai2_test/cmd_vel", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/depthai2_test/status", 10
        )
        self.event_publisher = self.create_publisher(
            String, "/depthai2_test/event", 10
        )
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        self.create_subscription(Bool, "/safety/stop", self._bumper_callback, 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.arm_service = self.create_service(
            Trigger, "/depthai2_test/arm", self._arm_callback
        )

        self.state = "DISARMED"
        self.stop_reason = "waiting_for_preflight"
        self.failure = False
        self.started_at = time.monotonic()
        self.arm_time = None
        self.finish_started_at = None
        self.last_status_at = 0.0
        self.last_health_at = 0.0

        self.last_camera_at = None
        self.last_target_at = None
        self.target_stable_since = None
        self.target_count = 0
        self.target_confidence = None
        self.target_center_x = None
        self.target_distance_m = None
        self.target_currently_valid = False
        self.target_issue = "waiting_for_target"
        self.last_person_debug = None
        self.raw_target_distance_m = None
        self.pending_jump_distance_m = None

        self.last_scan_at = None
        self.front_lidar_m = math.inf
        self.front_lidar_ray_count = 0
        self.last_bumper_at = None
        self.bumper_pressed = False
        self.last_odom_at = None
        self.odom_xy = None
        self.odom_yaw = None
        self.start_xy = None
        self.start_yaw = None

        self.pi_temp_c = None
        self.oak_temp_c = None
        self.throttled_value = None
        self.usb_speed = "unknown"
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.maximum_commanded_linear = 0.0
        self.maximum_commanded_angular = 0.0
        self.nonzero_command_count = 0
        self.last_motion_log_at = 0.0

        self.device = None
        self.detection_queue = None
        self.get_logger().info(
            "Starting isolated DepthAI %s spatial pipeline; motors remain disarmed."
            % self.REQUIRED_DEPTHAI_VERSION
        )
        self._start_pipeline()
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz, self._control_callback
        )
        self._publish_event("pipeline_started")

    def _declare_parameters(self):
        self.declare_parameter("blob_path", "")
        self.declare_parameter("blob_sha256", "")
        self.declare_parameter("motion_enabled", False)
        self.declare_parameter("allow_hot_test", False)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("camera_fps", 15.0)
        self.declare_parameter("startup_grace_sec", 6.0)
        self.declare_parameter("maximum_wait_sec", 35.0)
        self.declare_parameter("stable_target_sec", 1.0)
        self.declare_parameter("session_duration_sec", 10.0)
        self.declare_parameter("shutdown_zero_hold_sec", 0.5)
        self.declare_parameter("minimum_confidence", 0.60)
        self.declare_parameter("minimum_depth_m", 0.80)
        self.declare_parameter("maximum_depth_m", 3.50)
        self.declare_parameter("maximum_depth_jump_m", 0.60)
        self.declare_parameter("target_timeout_sec", 0.35)
        self.declare_parameter("camera_timeout_sec", 0.50)
        self.declare_parameter("sensor_timeout_sec", 0.30)
        self.declare_parameter("bumper_timeout_sec", 0.20)
        self.declare_parameter("front_sector_degrees", 60.0)
        self.declare_parameter("lidar_stop_distance_m", 0.60)
        self.declare_parameter("target_distance_m", 1.50)
        self.declare_parameter("distance_deadband_m", 0.25)
        self.declare_parameter("center_deadband", 0.10)
        self.declare_parameter("forward_center_limit", 0.15)
        self.declare_parameter("maximum_linear_mps", 0.05)
        self.declare_parameter("maximum_angular_rps", 0.15)
        self.declare_parameter("maximum_translation_m", 0.50)
        self.declare_parameter("maximum_yaw_rad", 0.35)
        self.declare_parameter("maximum_start_pi_temp_c", 70.0)
        self.declare_parameter("maximum_run_pi_temp_c", 75.0)
        self.declare_parameter("maximum_oak_temp_c", 85.0)

    def _load_parameters(self):
        def value(name):
            return self.get_parameter(name).value

        self.blob_path = Path(str(value("blob_path"))).expanduser()
        self.blob_sha256 = str(value("blob_sha256")).strip().lower()
        self.motion_enabled = bool(value("motion_enabled"))
        self.allow_hot_test = bool(value("allow_hot_test"))
        self.control_rate_hz = float(value("control_rate_hz"))
        self.camera_fps = float(value("camera_fps"))
        self.startup_grace_sec = float(value("startup_grace_sec"))
        self.maximum_wait_sec = float(value("maximum_wait_sec"))
        self.stable_target_sec = float(value("stable_target_sec"))
        self.session_duration_sec = float(value("session_duration_sec"))
        self.shutdown_zero_hold_sec = float(value("shutdown_zero_hold_sec"))
        self.minimum_confidence = float(value("minimum_confidence"))
        self.minimum_depth_m = float(value("minimum_depth_m"))
        self.maximum_depth_m = float(value("maximum_depth_m"))
        self.maximum_depth_jump_m = float(value("maximum_depth_jump_m"))
        self.target_timeout_sec = float(value("target_timeout_sec"))
        self.camera_timeout_sec = float(value("camera_timeout_sec"))
        self.sensor_timeout_sec = float(value("sensor_timeout_sec"))
        self.bumper_timeout_sec = float(value("bumper_timeout_sec"))
        self.front_sector_radians = math.radians(
            float(value("front_sector_degrees")) / 2.0
        )
        self.lidar_stop_distance_m = float(value("lidar_stop_distance_m"))
        self.limits = FollowLimits(
            target_distance_m=float(value("target_distance_m")),
            distance_deadband_m=float(value("distance_deadband_m")),
            center_deadband=float(value("center_deadband")),
            forward_center_limit=float(value("forward_center_limit")),
            max_linear_mps=float(value("maximum_linear_mps")),
            max_angular_rps=float(value("maximum_angular_rps")),
        )
        self.maximum_translation_m = float(value("maximum_translation_m"))
        self.maximum_yaw_rad = float(value("maximum_yaw_rad"))
        self.maximum_start_pi_temp_c = float(value("maximum_start_pi_temp_c"))
        self.maximum_run_pi_temp_c = float(value("maximum_run_pi_temp_c"))
        self.maximum_oak_temp_c = float(value("maximum_oak_temp_c"))

        if self.allow_hot_test:
            self.get_logger().warning(
                "ONE-SHOT HOT TEST OVERRIDE ENABLED: Raspberry Pi soft-temperature "
                "and heat-throttling gates are bypassed; undervoltage and the 90 C "
                "absolute stop remain active."
            )

    def _validate_runtime(self):
        if str(getattr(dai, "__version__", "")) != self.REQUIRED_DEPTHAI_VERSION:
            raise RuntimeError(
                "This test requires isolated depthai==%s, got %s"
                % (self.REQUIRED_DEPTHAI_VERSION, getattr(dai, "__version__", "unknown"))
            )
        if not self.blob_path.is_file():
            raise RuntimeError("MobileNet blob does not exist: %s" % self.blob_path)
        digest = hashlib.sha256(self.blob_path.read_bytes()).hexdigest()
        if self.blob_sha256 and digest != self.blob_sha256:
            raise RuntimeError(
                "MobileNet blob checksum mismatch: expected %s, got %s"
                % (self.blob_sha256, digest)
            )
        positive_values = (
            self.control_rate_hz,
            self.camera_fps,
            self.session_duration_sec,
            self.limits.max_linear_mps,
            self.limits.max_angular_rps,
        )
        if any(value <= 0.0 for value in positive_values):
            raise ValueError("Rates, duration, and motion limits must be positive")
        if self.session_duration_sec > 30.0:
            raise ValueError("This one-shot test may not exceed 30 seconds")
        if self.limits.max_linear_mps > 0.06:
            raise ValueError("Floor-test forward speed may not exceed 0.06 m/s")
        if self.limits.max_angular_rps > 0.20:
            raise ValueError("Floor-test turn rate may not exceed 0.20 rad/s")

    def _start_pipeline(self):
        pipeline = dai.Pipeline()
        rgb = pipeline.create(dai.node.ColorCamera)
        left = pipeline.create(dai.node.MonoCamera)
        right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        detector = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
        xout = pipeline.create(dai.node.XLinkOut)

        rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb.setPreviewSize(300, 300)
        rgb.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_1080_P
        )
        rgb.setInterleaved(False)
        rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        rgb.setFps(self.camera_fps)

        left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        left.setFps(self.camera_fps)
        right.setFps(self.camera_fps)

        stereo.setDefaultProfilePreset(
            dai.node.StereoDepth.PresetMode.HIGH_DENSITY
        )
        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setSubpixel(True)

        detector.setBlobPath(str(self.blob_path))
        detector.setConfidenceThreshold(self.minimum_confidence)
        detector.input.setBlocking(False)
        detector.setBoundingBoxScaleFactor(0.35)
        detector.setDepthLowerThreshold(int(self.minimum_depth_m * 1000.0))
        detector.setDepthUpperThreshold(int(self.maximum_depth_m * 1000.0))
        detector.setSpatialCalculationAlgorithm(
            dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        )

        xout.setStreamName("detections")
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        rgb.preview.link(detector.input)
        stereo.depth.link(detector.inputDepth)
        detector.out.link(xout.input)

        self.device = dai.Device(pipeline)
        self.detection_queue = self.device.getOutputQueue(
            "detections", maxSize=1, blocking=False
        )
        try:
            self.usb_speed = str(self.device.getUsbSpeed())
        except (AttributeError, RuntimeError):
            self.usb_speed = "unknown"

    def _control_callback(self):
        now = time.monotonic()
        self._poll_health(now)
        try:
            self._poll_detection(now)
        except Exception as error:  # DepthAI raises several runtime subclasses.
            self.get_logger().error("DepthAI queue failure: %s" % error)
            self._begin_finish("camera_exception", failed=True)

        if self.state == "DISARMED":
            self._publish_zero()
            if now - self.started_at > self.maximum_wait_sec:
                self._begin_finish("arm_timeout", failed=True)
            elif (
                self.last_camera_at is None
                and now - self.started_at > self.startup_grace_sec
            ):
                self._begin_finish("camera_start_timeout", failed=True)
        elif self.state == "ARMED":
            reason = self._runtime_block_reason(now)
            if reason is not None:
                self._begin_finish(reason, failed=True)
            elif now - self.arm_time >= self.session_duration_sec:
                self._begin_finish("completed_bounded_test", failed=False)
            else:
                linear_x, angular_z = bounded_follow_command(
                    self.target_center_x,
                    self.target_distance_m,
                    self.limits,
                )
                self._publish_command(linear_x, angular_z)
                if now - self.last_motion_log_at >= 1.0:
                    travel, yaw = self._motion_totals()
                    self.get_logger().info(
                        "MOTION t=%.1fs target=%.2fm center=%.3f "
                        "cmd=(%.3fm/s, %.3frad/s) odom=(%.3fm, %.3frad) "
                        "lidar=%.3fm pi=%.1fC"
                        % (
                            now - self.arm_time,
                            self.target_distance_m,
                            self.target_center_x,
                            self.last_linear,
                            self.last_angular,
                            travel,
                            yaw,
                            self.front_lidar_m,
                            self.pi_temp_c,
                        )
                    )
                    self.last_motion_log_at = now
        else:
            self._publish_zero()
            if now - self.finish_started_at >= self.shutdown_zero_hold_sec:
                if rclpy.ok(context=self.context):
                    rclpy.shutdown(context=self.context)

        if now - self.last_status_at >= 0.20:
            self._publish_status(now)
            self.last_status_at = now

    def _poll_detection(self, now):
        latest = None
        while self.detection_queue is not None:
            packet = self.detection_queue.tryGet()
            if packet is None:
                break
            latest = packet
        if latest is None:
            return

        self.last_camera_at = now
        candidates = []
        person_debug = []
        person_count = 0
        for detection in latest.detections:
            if int(detection.label) != self.PERSON_LABEL:
                continue
            confidence = float(detection.confidence)
            if confidence < self.minimum_confidence:
                continue
            person_count += 1
            distance_m = float(detection.spatialCoordinates.z) / 1000.0
            center_x = (float(detection.xmin) + float(detection.xmax)) / 2.0
            bbox_valid = (
                0.01 < float(detection.xmin) < float(detection.xmax) < 0.99
                and 0.0 <= float(detection.ymin) < float(detection.ymax) <= 1.0
            )
            depth_valid = (
                math.isfinite(distance_m)
                and self.minimum_depth_m <= distance_m <= self.maximum_depth_m
            )
            person_debug.append({
                "confidence": round(confidence, 3),
                "distance_m": (
                    round(distance_m, 3) if math.isfinite(distance_m) else None
                ),
                "center_x": round(center_x, 3),
                "bbox": [
                    round(float(detection.xmin), 3),
                    round(float(detection.ymin), 3),
                    round(float(detection.xmax), 3),
                    round(float(detection.ymax), 3),
                ],
                "bbox_valid": bbox_valid,
                "depth_valid": depth_valid,
            })
            if (
                depth_valid
                and 0.0 <= center_x <= 1.0
                and bbox_valid
            ):
                candidates.append((confidence, center_x, distance_m))

        self.target_count = person_count
        self.last_person_debug = person_debug
        if person_count != 1 or len(candidates) != 1:
            reason = "target_lost"
            if person_count > 1:
                reason = "multiple_people"
            elif person_count == 1 and person_debug:
                if not person_debug[0]["depth_valid"]:
                    reason = "target_depth_out_of_range"
                elif not person_debug[0]["bbox_valid"]:
                    reason = "target_bbox_cropped"
            self._invalidate_target(
                reason
            )
            return

        confidence, center_x, raw_distance = candidates[0]
        if self.raw_target_distance_m is not None and abs(
            raw_distance - self.raw_target_distance_m
        ) > self.maximum_depth_jump_m:
            if (
                self.pending_jump_distance_m is None
                or abs(raw_distance - self.pending_jump_distance_m) > 0.20
            ):
                self.pending_jump_distance_m = raw_distance
                self.target_currently_valid = False
                self.target_issue = "depth_jump_unconfirmed"
                return
        self.pending_jump_distance_m = None

        if self.target_stable_since is None:
            self.target_stable_since = now
        alpha = 0.35
        if self.target_center_x is None:
            self.target_center_x = center_x
            self.target_distance_m = raw_distance
        else:
            self.target_center_x = (
                alpha * center_x + (1.0 - alpha) * self.target_center_x
            )
            self.target_distance_m = (
                alpha * raw_distance
                + (1.0 - alpha) * self.target_distance_m
            )
        self.raw_target_distance_m = raw_distance
        self.target_confidence = confidence
        self.last_target_at = now
        self.target_currently_valid = True
        self.target_issue = None

    def _invalidate_target(self, reason):
        self.target_stable_since = None
        self.target_confidence = None
        self.target_center_x = None
        self.target_distance_m = None
        self.target_currently_valid = False
        self.target_issue = reason
        self.raw_target_distance_m = None
        self.pending_jump_distance_m = None

    def _scan_callback(self, message):
        front = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            angle = normalize_angle(angle)
            if abs(angle) > self.front_sector_radians:
                continue
            if not math.isfinite(distance):
                continue
            if message.range_min <= distance <= message.range_max:
                front.append(float(distance))
        self.front_lidar_ray_count = len(front)
        self.front_lidar_m = min(front) if front else math.inf
        self.last_scan_at = time.monotonic()

    def _bumper_callback(self, message):
        self.bumper_pressed = bool(message.data)
        self.last_bumper_at = time.monotonic()

    def _odom_callback(self, message):
        self.last_odom_at = time.monotonic()
        self.odom_xy = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        q = message.pose.pose.orientation
        self.odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _poll_health(self, now):
        if now - self.last_health_at < 0.50:
            return
        self.last_health_at = now
        try:
            self.pi_temp_c = float(
                Path("/sys/class/thermal/thermal_zone0/temp").read_text().strip()
            ) / 1000.0
        except (OSError, ValueError):
            self.pi_temp_c = None
        try:
            temperature = self.device.getChipTemperature()
            self.oak_temp_c = float(temperature.average)
        except (AttributeError, RuntimeError, TypeError):
            self.oak_temp_c = None
        executable = shutil.which("vcgencmd")
        if executable is None:
            self.throttled_value = None
            return
        try:
            result = subprocess.run(
                [executable, "get_throttled"],
                check=True,
                capture_output=True,
                text=True,
                timeout=0.4,
            )
            self.throttled_value = int(result.stdout.strip().split("=", 1)[1], 16)
        except (OSError, ValueError, IndexError, subprocess.SubprocessError):
            self.throttled_value = None

    def _arm_callback(self, _request, response):
        now = time.monotonic()
        if self.state != "DISARMED":
            response.success = False
            response.message = "Test is already %s" % self.state.lower()
            return response
        reason = self._preflight_block_reason(now)
        if reason is not None:
            response.success = False
            response.message = "Not armed: %s" % reason
            self.stop_reason = reason
            self._publish_event("arm_rejected:%s" % reason)
            return response

        self.state = "ARMED"
        self.stop_reason = None
        self.arm_time = now
        self.start_xy = self.odom_xy
        self.start_yaw = self.odom_yaw
        self._publish_event("armed_10_second_floor_follow")
        self.get_logger().warning(
            "ARMED: bounded floor follow is live for at most %.1f seconds."
            % self.session_duration_sec
        )
        response.success = True
        response.message = "Armed for one bounded %.1f-second run" % (
            self.session_duration_sec
        )
        return response

    def _common_block_reason(self, now):
        if self.last_camera_at is None or now - self.last_camera_at > (
            self.camera_timeout_sec
        ):
            return "camera_stale"
        if self.target_count > 1:
            return "multiple_people"
        if not self.target_currently_valid:
            return self.target_issue or "target_lost"
        if self.last_target_at is None or now - self.last_target_at > (
            self.target_timeout_sec
        ):
            return "target_lost"
        if self.last_scan_at is None or now - self.last_scan_at > (
            self.sensor_timeout_sec
        ):
            return "lidar_stale"
        if self.front_lidar_ray_count <= 0:
            return "lidar_front_sector_empty"
        if self.front_lidar_m < self.lidar_stop_distance_m:
            return "lidar_obstacle"
        if self.last_bumper_at is None or now - self.last_bumper_at > (
            self.bumper_timeout_sec
        ):
            return "bumper_heartbeat_stale"
        if self.bumper_pressed:
            return "bumper_pressed"
        if self.last_odom_at is None or now - self.last_odom_at > (
            self.sensor_timeout_sec
        ):
            return "odom_stale"
        if self.pi_temp_c is None:
            return "pi_temperature_unavailable"
        if self.oak_temp_c is None:
            return "oak_temperature_unavailable"
        if self.throttled_value is None:
            return "power_status_unavailable"
        throttle_bits = current_throttle_bits(self.throttled_value)
        # Undervoltage (bit 0) remains fail-closed in every mode. The explicit
        # one-shot hot-test override may ignore only the heat-related frequency,
        # throttling, and soft-temperature flags (bits 1-3).
        if throttle_bits & 0x1:
            return "pi_power_or_thermal_throttle"
        if throttle_bits & 0xE and not self.allow_hot_test:
            return "pi_power_or_thermal_throttle"
        if self.oak_temp_c >= self.maximum_oak_temp_c:
            return "oak_over_temperature"
        return None

    def _preflight_block_reason(self, now):
        if not self.motion_enabled:
            return "motion_disabled"
        reason = self._common_block_reason(now)
        if reason is not None:
            return reason
        if not self.allow_hot_test and self.pi_temp_c >= self.maximum_start_pi_temp_c:
            return "pi_too_hot_to_arm"
        if self.target_stable_since is None or now - self.target_stable_since < (
            self.stable_target_sec
        ):
            return "target_not_stable"

        names = self.get_node_names()
        if names.count("motor_driver_node") != 1:
            return "motor_driver_ownership"
        if names.count("encoder_odom_node") != 1:
            return "encoder_ownership"
        if names.count("LD19") != 1:
            return "lidar_ownership"
        if names.count("bumper_node") != 1:
            return "bumper_ownership"
        if "person_follower_node" in names or "twist_mux" in names:
            return "competing_motion_stack"
        if self.count_publishers("/depthai2_test/cmd_vel") != 1:
            return "private_command_topic_ownership"
        if self.count_publishers("/safety/stop") != 1:
            return "safety_stop_topic_ownership"
        return None

    def _runtime_block_reason(self, now):
        reason = self._common_block_reason(now)
        if reason is not None:
            return reason
        if self.pi_temp_c >= 90.0:
            return "pi_absolute_over_temperature"
        if not self.allow_hot_test and self.pi_temp_c >= self.maximum_run_pi_temp_c:
            return "pi_over_temperature"
        if self.start_xy is None or self.start_yaw is None:
            return "odom_start_missing"
        if planar_distance(self.start_xy, self.odom_xy) >= self.maximum_translation_m:
            return "translation_budget_reached"
        if abs(normalize_angle(self.odom_yaw - self.start_yaw)) >= (
            self.maximum_yaw_rad
        ):
            return "yaw_budget_reached"
        return None

    def _publish_command(self, linear_x, angular_z):
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            self._begin_finish("nonfinite_command", failed=True)
            return
        linear_x = max(0.0, min(self.limits.max_linear_mps, linear_x))
        angular_z = max(
            -self.limits.max_angular_rps,
            min(self.limits.max_angular_rps, angular_z),
        )
        command = Twist()
        command.linear.x = float(linear_x)
        command.angular.z = float(angular_z)
        self.command_publisher.publish(command)
        self.last_linear = command.linear.x
        self.last_angular = command.angular.z
        self.maximum_commanded_linear = max(
            self.maximum_commanded_linear, self.last_linear
        )
        self.maximum_commanded_angular = max(
            self.maximum_commanded_angular, abs(self.last_angular)
        )
        if abs(self.last_linear) > 1.0e-6 or abs(self.last_angular) > 1.0e-6:
            self.nonzero_command_count += 1

    def _publish_zero(self):
        self.last_linear = 0.0
        self.last_angular = 0.0
        if not self.context.ok():
            return
        try:
            self.command_publisher.publish(Twist())
        except RCLError:
            pass

    def _begin_finish(self, reason, failed):
        if self.state in {"FINISHING", "FAULT"}:
            return
        self.state = "FAULT" if failed else "FINISHING"
        self.failure = bool(failed)
        self.stop_reason = reason
        self.finish_started_at = time.monotonic()
        self._publish_zero()
        self._publish_event(("fault:" if failed else "finished:") + reason)
        log = self.get_logger().error if failed else self.get_logger().info
        log("Stopping bounded floor test: %s" % reason)
        travel, yaw = self._motion_totals()
        log(
            "MOTION SUMMARY nonzero_commands=%d max_linear=%.3fm/s "
            "max_angular=%.3frad/s odom_translation=%.3fm odom_yaw=%.3frad"
            % (
                self.nonzero_command_count,
                self.maximum_commanded_linear,
                self.maximum_commanded_angular,
                travel,
                yaw,
            )
        )

    def _motion_totals(self):
        travel = (
            planar_distance(self.start_xy, self.odom_xy)
            if self.start_xy is not None and self.odom_xy is not None
            else 0.0
        )
        yaw = (
            abs(normalize_angle(self.odom_yaw - self.start_yaw))
            if self.start_yaw is not None and self.odom_yaw is not None
            else 0.0
        )
        return travel, yaw

    def _publish_event(self, event):
        if not self.context.ok():
            return
        try:
            self.event_publisher.publish(String(data=event))
        except (AttributeError, RCLError):
            pass

    @staticmethod
    def _age(now, timestamp):
        return None if timestamp is None else round(now - timestamp, 3)

    def _publish_status(self, now):
        preflight_reason = (
            self._preflight_block_reason(now)
            if self.state == "DISARMED"
            else None
        )
        travel = (
            planar_distance(self.start_xy, self.odom_xy)
            if self.start_xy is not None and self.odom_xy is not None
            else 0.0
        )
        yaw = (
            abs(normalize_angle(self.odom_yaw - self.start_yaw))
            if self.start_yaw is not None and self.odom_yaw is not None
            else 0.0
        )
        payload = {
            "test": "depthai2_bounded_floor_follow",
            "depthai_version": str(dai.__version__),
            "usb_speed": self.usb_speed,
            "state": self.state,
            "motion_enabled": self.motion_enabled,
            "hot_test_override": self.allow_hot_test,
            "ready": self.state == "DISARMED" and preflight_reason is None,
            "ready_blocked_by": preflight_reason,
            "stop_reason": self.stop_reason,
            "target_count": self.target_count,
            "target_currently_valid": self.target_currently_valid,
            "target_issue": self.target_issue,
            "target_confidence": self.target_confidence,
            "target_center_x": self.target_center_x,
            "target_distance_m": self.target_distance_m,
            "person_debug": self.last_person_debug,
            "camera_age_sec": self._age(now, self.last_camera_at),
            "target_age_sec": self._age(now, self.last_target_at),
            "target_stable_sec": (
                0.0
                if self.target_stable_since is None
                else round(now - self.target_stable_since, 3)
            ),
            "lidar_age_sec": self._age(now, self.last_scan_at),
            "front_lidar_m": (
                None if not math.isfinite(self.front_lidar_m) else self.front_lidar_m
            ),
            "front_lidar_rays": self.front_lidar_ray_count,
            "bumper_age_sec": self._age(now, self.last_bumper_at),
            "bumper_pressed": self.bumper_pressed,
            "odom_age_sec": self._age(now, self.last_odom_at),
            "pi_temp_c": self.pi_temp_c,
            "oak_temp_c": self.oak_temp_c,
            "throttled": (
                None
                if self.throttled_value is None
                else "0x%x" % self.throttled_value
            ),
            "elapsed_armed_sec": (
                0.0 if self.arm_time is None else round(now - self.arm_time, 3)
            ),
            "translation_m": round(travel, 4),
            "yaw_rad": round(yaw, 4),
            "command_linear_mps": self.last_linear,
            "command_angular_rps": self.last_angular,
            "nonzero_command_count": self.nonzero_command_count,
            "maximum_commanded_linear_mps": self.maximum_commanded_linear,
            "maximum_commanded_angular_rps": self.maximum_commanded_angular,
        }
        try:
            self.status_publisher.publish(String(data=json.dumps(payload)))
        except RCLError:
            pass

    def destroy_node(self):
        try:
            self._publish_zero()
            if self.device is not None:
                try:
                    self.device.close()
                except RuntimeError:
                    pass
                self.device = None
                self.detection_queue = None
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    failure = True
    try:
        node = DepthAI2FloorFollowTestNode()
        rclpy.spin(node)
        failure = node.failure
    except KeyboardInterrupt:
        failure = True
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failure:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
