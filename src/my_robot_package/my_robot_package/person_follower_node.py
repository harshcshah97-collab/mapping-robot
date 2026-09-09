#!/usr/bin/env python3
"""OAK-D person tracking, safe following, framing, and recording."""

import hashlib
import json
import math
import shutil
import subprocess
import time
from datetime import datetime
from pathlib import Path

import cv2
import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String

from my_robot_package.appearance_logic import (
    HISTOGRAM_BINS,
    appearance_distance,
    appearance_signature,
    roi_bounds,
)
from my_robot_package.tracking_logic import compute_motion


class PersonFollowerNode(Node):
    """Track one person with OAK-D spatial detections and command the base safely."""

    PERSON_LABEL = 15
    REQUIRED_DEPTHAI_VERSION = "2.32.0.0"
    VALID_MODES = {"IDLE", "FOLLOW", "KEEP_FRAME"}

    def __init__(self):
        super().__init__("person_follower_node")

        self._declare_parameters()
        self._load_parameters()

        self.mode = "IDLE"
        self.locked_target_id = None
        self.smoothed_x = 0.5
        self.smoothed_distance = self.target_distance_m
        self.last_target_time = None
        self.last_frame = None
        self.last_target_bounds = None
        self.enrollment_pending = False
        self.enrolled_signature = self._load_enrollment()
        self.visible_candidate_count = 0
        self.photo_pending = False
        self.video_writer = None
        self.recording_path = None
        self.recording_requested = False
        self.recording_start_time = None
        self.last_status_time = 0.0
        self.last_block_reason = None

        self.front_lidar_distance = math.inf
        self.last_lidar_time = None
        self.range_readings = {}
        self.last_bumper_time = None
        self.last_safety_stop_time = None
        self.bumper_pressed = False
        self.pi_temp_c = None
        self.oak_temp_c = None
        self.throttled_value = None
        self.last_health_time = 0.0
        self.last_motion_log_time = 0.0

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.image_publisher = self.create_publisher(Image, "/oakd/color/image_raw", 5)
        self.status_publisher = self.create_publisher(String, "/tracking/status", 10)
        self.event_publisher = self.create_publisher(String, "/tracking/event", 10)
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, "/oakd/depth/points", 5
        )

        self.create_subscription(String, "/tracking/command", self.command_callback, 10)
        # Temporary compatibility with the old assistant command topic.
        self.create_subscription(String, "/assistant/command", self.command_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Range, "/ultrasonic_distance", self.ultrasonic_callback, 10)
        self.create_subscription(Range, "/ir/left", self.ir_left_callback, 10)
        self.create_subscription(Range, "/ir/right", self.ir_right_callback, 10)
        self.create_subscription(PointCloud2, "/bumper_cloud", self.bumper_callback, 10)
        self.create_subscription(Bool, "/safety/stop", self.safety_stop_callback, 10)

        self.cv_bridge = CvBridge()
        self.get_logger().info("Initializing OAK-D DepthAI 2.32 spatial tracker...")
        self._create_pipeline()

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.timer_callback)
        self.set_mode(self.initial_mode)
        self.get_logger().info(
            "Tracker ready. Commands: start_follow, keep_frame, stop, "
            "enroll_target, clear_enrollment, take_photo, start_recording, "
            "stop_recording."
        )

    def _declare_parameters(self):
        self.declare_parameter("initial_mode", "idle")
        self.declare_parameter("blob_path", "")
        self.declare_parameter("blob_sha256", "")
        self.declare_parameter("target_distance_m", 1.5)
        self.declare_parameter("distance_deadband_m", 0.20)
        self.declare_parameter("center_deadband", 0.10)
        self.declare_parameter("smoothing_alpha", 0.30)
        self.declare_parameter("turn_speed", 0.40)
        self.declare_parameter("follow_speed", 0.12)
        self.declare_parameter("control_rate_hz", 15.0)
        self.declare_parameter("target_timeout_sec", 0.8)
        self.declare_parameter("minimum_valid_depth_m", 0.30)
        self.declare_parameter("maximum_valid_depth_m", 6.0)
        self.declare_parameter("front_sector_degrees", 50.0)
        self.declare_parameter("lidar_stop_distance_m", 0.40)
        self.declare_parameter("range_stop_distance_m", 0.30)
        self.declare_parameter("sensor_timeout_sec", 1.0)
        self.declare_parameter("bumper_hold_sec", 1.0)
        self.declare_parameter("require_lidar", True)
        self.declare_parameter("require_ultrasonic", True)
        self.declare_parameter("require_ir", True)
        self.declare_parameter("require_bumper", True)
        self.declare_parameter("ir_stop_requires_both", False)
        self.declare_parameter("motion_enabled", True)
        self.declare_parameter("allow_hot_operation", False)
        self.declare_parameter("maximum_run_pi_temp_c", 75.0)
        self.declare_parameter("absolute_stop_pi_temp_c", 95.0)
        self.declare_parameter("maximum_oak_temp_c", 85.0)
        self.declare_parameter("publish_preview", True)
        self.declare_parameter("publish_pointcloud", False)
        self.declare_parameter("oakd_extrinsics_calibrated", False)
        self.declare_parameter("oakd_depth_benchmark_passed", False)
        self.declare_parameter("pointcloud_benchmark_mode", False)
        self.declare_parameter("pointcloud_decimation", 8)
        self.declare_parameter("recording_fps", 15.0)
        self.declare_parameter("max_recording_duration_sec", 600.0)
        self.declare_parameter("minimum_free_disk_mb", 512.0)
        self.declare_parameter("recording_directory", "~/robot_recordings")
        self.declare_parameter("reidentification_enabled", True)
        self.declare_parameter("require_enrollment_for_motion", True)
        self.declare_parameter("reid_match_threshold", 0.35)
        self.declare_parameter(
            "enrollment_file", "~/.config/mapping-robot/target_enrollment.json"
        )

    def _load_parameters(self):
        self.initial_mode = str(self.get_parameter("initial_mode").value)
        self.blob_path = Path(
            str(self.get_parameter("blob_path").value)
        ).expanduser()
        self.blob_sha256 = str(
            self.get_parameter("blob_sha256").value
        ).strip().lower()
        self.target_distance_m = float(self.get_parameter("target_distance_m").value)
        self.distance_deadband_m = float(
            self.get_parameter("distance_deadband_m").value
        )
        self.center_deadband = float(self.get_parameter("center_deadband").value)
        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.follow_speed = float(self.get_parameter("follow_speed").value)
        self.control_rate_hz = max(
            1.0, float(self.get_parameter("control_rate_hz").value)
        )
        self.target_timeout_sec = float(
            self.get_parameter("target_timeout_sec").value
        )
        self.minimum_valid_depth_m = float(
            self.get_parameter("minimum_valid_depth_m").value
        )
        self.maximum_valid_depth_m = float(
            self.get_parameter("maximum_valid_depth_m").value
        )
        self.front_sector_radians = math.radians(
            float(self.get_parameter("front_sector_degrees").value) / 2.0
        )
        self.lidar_stop_distance_m = float(
            self.get_parameter("lidar_stop_distance_m").value
        )
        self.range_stop_distance_m = float(
            self.get_parameter("range_stop_distance_m").value
        )
        self.sensor_timeout_sec = float(
            self.get_parameter("sensor_timeout_sec").value
        )
        self.bumper_hold_sec = float(self.get_parameter("bumper_hold_sec").value)
        self.require_lidar = bool(self.get_parameter("require_lidar").value)
        self.require_ultrasonic = bool(
            self.get_parameter("require_ultrasonic").value
        )
        self.require_ir = bool(self.get_parameter("require_ir").value)
        self.require_bumper = bool(self.get_parameter("require_bumper").value)
        self.ir_stop_requires_both = bool(
            self.get_parameter("ir_stop_requires_both").value
        )
        self.motion_enabled = bool(self.get_parameter("motion_enabled").value)
        self.allow_hot_operation = bool(
            self.get_parameter("allow_hot_operation").value
        )
        self.maximum_run_pi_temp_c = float(
            self.get_parameter("maximum_run_pi_temp_c").value
        )
        self.absolute_stop_pi_temp_c = float(
            self.get_parameter("absolute_stop_pi_temp_c").value
        )
        self.maximum_oak_temp_c = float(
            self.get_parameter("maximum_oak_temp_c").value
        )
        self.publish_preview = bool(self.get_parameter("publish_preview").value)
        requested_pointcloud = bool(
            self.get_parameter("publish_pointcloud").value
        )
        self.oakd_extrinsics_calibrated = bool(
            self.get_parameter("oakd_extrinsics_calibrated").value
        )
        self.oakd_depth_benchmark_passed = bool(
            self.get_parameter("oakd_depth_benchmark_passed").value
        )
        self.pointcloud_benchmark_mode = bool(
            self.get_parameter("pointcloud_benchmark_mode").value
        )
        self.publish_pointcloud = (
            requested_pointcloud
            and self.oakd_extrinsics_calibrated
            and (
                self.oakd_depth_benchmark_passed
                or self.pointcloud_benchmark_mode
            )
        )
        self.pointcloud_decimation = max(
            1, int(self.get_parameter("pointcloud_decimation").value)
        )
        if requested_pointcloud and not self.publish_pointcloud:
            missing = []
            if not self.oakd_extrinsics_calibrated:
                missing.append("measured OAK-D mount transform")
            if (
                not self.oakd_depth_benchmark_passed
                and not self.pointcloud_benchmark_mode
            ):
                missing.append("passing depth benchmark")
            self.get_logger().error(
                "OAK-D depth was requested, but it still needs: %s. Point-cloud "
                "publication is disabled." % ", ".join(missing)
            )
        self.recording_fps = float(self.get_parameter("recording_fps").value)
        self.max_recording_duration_sec = float(
            self.get_parameter("max_recording_duration_sec").value
        )
        self.minimum_free_disk_mb = float(
            self.get_parameter("minimum_free_disk_mb").value
        )
        self.recording_directory = Path(
            str(self.get_parameter("recording_directory").value)
        ).expanduser()
        self.reidentification_enabled = bool(
            self.get_parameter("reidentification_enabled").value
        )
        self.require_enrollment_for_motion = bool(
            self.get_parameter("require_enrollment_for_motion").value
        )
        self.reid_match_threshold = float(
            self.get_parameter("reid_match_threshold").value
        )
        self.enrollment_file = Path(
            str(self.get_parameter("enrollment_file").value)
        ).expanduser()
        if not 0.0 <= self.reid_match_threshold <= 1.0:
            raise ValueError("reid_match_threshold must be between 0 and 1")
        if str(getattr(dai, "__version__", "")) != self.REQUIRED_DEPTHAI_VERSION:
            raise RuntimeError(
                "Normal tracking requires isolated depthai==%s, got %s"
                % (
                    self.REQUIRED_DEPTHAI_VERSION,
                    getattr(dai, "__version__", "unknown"),
                )
            )
        if not self.blob_path.is_file():
            raise RuntimeError("MobileNet blob does not exist: %s" % self.blob_path)
        digest = hashlib.sha256(self.blob_path.read_bytes()).hexdigest()
        if self.blob_sha256 and digest != self.blob_sha256:
            raise RuntimeError(
                "MobileNet blob checksum mismatch: expected %s, got %s"
                % (self.blob_sha256, digest)
            )
        if self.allow_hot_operation:
            self.get_logger().warning(
                "HOT OPERATION OVERRIDE ENABLED: heat-throttling gates are "
                "bypassed, but undervoltage and the %.1f C absolute stop remain."
                % self.absolute_stop_pi_temp_c
            )

    def _load_enrollment(self):
        if not self.reidentification_enabled or not self.enrollment_file.is_file():
            return None
        try:
            payload = json.loads(self.enrollment_file.read_text())
            if tuple(payload.get("bins", [])) != HISTOGRAM_BINS:
                raise ValueError("histogram layout does not match this software")
            signature = np.asarray(payload["signature"], dtype=np.float32)
            if signature.size != HISTOGRAM_BINS[0] * HISTOGRAM_BINS[1]:
                raise ValueError("signature has the wrong size")
            self.get_logger().info(
                "Loaded enrolled subject appearance from %s" % self.enrollment_file
            )
            return signature
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                "Ignoring invalid enrollment file %s: %s"
                % (self.enrollment_file, error)
            )
            return None

    def _create_pipeline(self):
        """Build the proven DepthAI 2.32 RGB + stereo tracking pipeline."""
        self.pipeline = dai.Pipeline()
        rgb_camera = self.pipeline.create(dai.node.ColorCamera)
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        detector = self.pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
        tracker = self.pipeline.create(dai.node.ObjectTracker)
        rgb_output = self.pipeline.create(dai.node.XLinkOut)
        tracklets_output = self.pipeline.create(dai.node.XLinkOut)

        rgb_camera.setBoardSocket(dai.CameraBoardSocket.RGB)
        rgb_camera.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_1080_P
        )
        rgb_camera.setPreviewSize(300, 300)
        rgb_camera.setInterleaved(False)
        rgb_camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        rgb_camera.setFps(self.control_rate_hz)

        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        for camera in (mono_left, mono_right):
            camera.setResolution(
                dai.MonoCameraProperties.SensorResolution.THE_480_P
            )
            camera.setFps(self.control_rate_hz)

        stereo.setDefaultProfilePreset(
            dai.node.StereoDepth.PresetMode.HIGH_DENSITY
        )
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

        detector.setBlobPath(str(self.blob_path))
        detector.setConfidenceThreshold(0.60)
        detector.input.setBlocking(False)
        detector.setBoundingBoxScaleFactor(0.35)
        detector.setDepthLowerThreshold(int(self.minimum_valid_depth_m * 1000.0))
        detector.setDepthUpperThreshold(int(self.maximum_valid_depth_m * 1000.0))
        detector.setSpatialCalculationAlgorithm(
            dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        )

        tracker.setDetectionLabelsToTrack([self.PERSON_LABEL])
        tracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
        tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.UNIQUE_ID)

        rgb_output.setStreamName("rgb")
        tracklets_output.setStreamName("tracklets")
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        rgb_camera.preview.link(detector.input)
        stereo.depth.link(detector.inputDepth)
        detector.passthrough.link(tracker.inputTrackerFrame)
        detector.passthrough.link(tracker.inputDetectionFrame)
        detector.out.link(tracker.inputDetections)
        detector.passthrough.link(rgb_output.input)
        tracker.out.link(tracklets_output.input)

        self.device = dai.Device(self.pipeline)
        self.rgb_queue = self.device.getOutputQueue(
            "rgb", maxSize=2, blocking=False
        )
        self.tracklets_queue = self.device.getOutputQueue(
            "tracklets", maxSize=2, blocking=False
        )
        self.pointcloud_queue = None
        if self.publish_pointcloud:
            self.get_logger().warning(
                "Point-cloud output is disabled in the stable DepthAI 2 tracking "
                "path; spatial person depth remains active."
            )
            self.publish_pointcloud = False

    def command_callback(self, message):
        command = message.data.strip().lower()
        aliases = {
            "start_follow": "FOLLOW",
            "start_follow:behind": "FOLLOW",
            "follow": "FOLLOW",
            "keep_frame": "KEEP_FRAME",
            "start_follow:frame": "KEEP_FRAME",
            "stop": "IDLE",
            "stop_follow": "IDLE",
            "idle": "IDLE",
        }

        if command in aliases:
            requested_mode = aliases[command]
            if requested_mode != "IDLE" and not self.motion_enabled:
                self._publish_event("motion_unavailable:perception_only")
                self.get_logger().warning(
                    "Ignoring motion command because this is a perception-only launch."
                )
            elif (
                requested_mode != "IDLE"
                and self.require_enrollment_for_motion
                and self.enrolled_signature is None
            ):
                self.stop_robot()
                self._publish_event("motion_blocked:target_not_enrolled")
                self.get_logger().warning(
                    "Enroll the intended subject before enabling tracking motion."
                )
            else:
                self.set_mode(requested_mode)
        elif command in {"orbit", "orbit_left", "orbit_right", "start_follow:orbit"}:
            self.stop_robot()
            self._publish_event(
                "orbit_unavailable:requires_velocity_control_or_camera_gimbal"
            )
            self.get_logger().warning(
                "Orbit is disabled: a fixed forward camera on a differential-drive "
                "base cannot stay aimed inward while moving tangentially."
            )
        elif command == "take_photo":
            self.photo_pending = True
        elif command == "enroll_target":
            self._request_enrollment()
        elif command == "clear_enrollment":
            self._clear_enrollment()
        elif command == "start_recording":
            self.start_recording()
        elif command == "stop_recording":
            self.stop_recording()
        else:
            self.get_logger().warning("Unknown tracking command: %s" % command)

    def set_mode(self, mode):
        normalized = str(mode).strip().upper()
        if normalized == "BEHIND":
            normalized = "FOLLOW"
        if normalized not in self.VALID_MODES:
            self.get_logger().warning(
                "Invalid initial mode '%s'; remaining IDLE." % mode
            )
            normalized = "IDLE"
        if normalized != "IDLE" and not self.motion_enabled:
            self.get_logger().warning(
                "Motion is disabled for this launch; forcing the tracker to IDLE."
            )
            normalized = "IDLE"

        self.mode = normalized
        self.locked_target_id = None
        self.last_target_time = None
        self.smoothed_x = 0.5
        self.smoothed_distance = self.target_distance_m
        self.stop_robot()
        self._publish_event("mode:%s" % self.mode.lower())
        self.get_logger().info("Tracking mode set to %s" % self.mode)

    def timer_callback(self):
        self._poll_health()
        frame_packet = self._drain_latest(self.rgb_queue)
        tracklets_packet = self._drain_latest(self.tracklets_queue)
        pointcloud_packet = (
            self._drain_latest(self.pointcloud_queue)
            if self.pointcloud_queue is not None
            else None
        )

        if pointcloud_packet is not None:
            self._publish_pointcloud(pointcloud_packet)

        if frame_packet is not None:
            self.last_frame = frame_packet.getCvFrame()
            if self.publish_preview:
                image_message = self.cv_bridge.cv2_to_imgmsg(
                    self.last_frame, encoding="bgr8"
                )
                image_message.header.stamp = self.get_clock().now().to_msg()
                image_message.header.frame_id = "oakd_rgb_camera_optical_frame"
                self.image_publisher.publish(image_message)
            if self.recording_requested and self.video_writer is None:
                self._open_recording_writer()
            if self.video_writer is not None:
                self.video_writer.write(self.last_frame)
                if time.monotonic() - self.recording_start_time > (
                    self.max_recording_duration_sec
                ):
                    self.get_logger().info("Maximum recording duration reached.")
                    self.stop_recording()

        if self.photo_pending and self.last_frame is not None:
            self.save_photo()

        target = None
        if tracklets_packet is not None:
            target = self._select_target(tracklets_packet.tracklets)

        if target is not None:
            observation = self._target_observation(target)
            if observation is not None:
                if self.last_frame is not None:
                    self.last_target_bounds = roi_bounds(
                        target.roi, self.last_frame.shape
                    )
                    if self.enrollment_pending:
                        self._save_enrollment(self.last_target_bounds)
                center_x, distance_m = observation
                alpha = self.smoothing_alpha
                self.smoothed_x = alpha * center_x + (1.0 - alpha) * self.smoothed_x
                self.smoothed_distance = (
                    alpha * distance_m + (1.0 - alpha) * self.smoothed_distance
                )
                self.last_target_time = time.monotonic()

        if self.mode == "IDLE":
            self.stop_robot()
        elif self._target_is_stale():
            self.stop_robot()
            self.locked_target_id = None
            self._set_block_reason("target_lost")
        else:
            self._control_robot()

        self._publish_status_periodically()

    @staticmethod
    def _drain_latest(queue):
        latest = None
        while True:
            packet = queue.tryGet()
            if packet is None:
                return latest
            latest = packet

    def _select_target(self, tracklets):
        candidates = []
        for tracklet in tracklets:
            status_name = getattr(tracklet.status, "name", str(tracklet.status))
            if status_name in {"LOST", "REMOVED"}:
                continue
            if int(tracklet.label) != self.PERSON_LABEL:
                continue
            observation = self._target_observation(tracklet)
            if observation is None:
                continue
            signature = None
            if self.last_frame is not None and self.reidentification_enabled:
                bounds = roi_bounds(tracklet.roi, self.last_frame.shape)
                signature = appearance_signature(self.last_frame, bounds)
            candidates.append((tracklet, observation[1], signature))

        self.visible_candidate_count = len(candidates)
        if self.enrollment_pending and len(candidates) != 1:
            self._set_block_reason("enrollment_requires_exactly_one_person")
            return None

        for tracklet, _distance, signature in candidates:
            if self.locked_target_id is not None and tracklet.id == self.locked_target_id:
                if self.enrolled_signature is not None:
                    if signature is None or appearance_distance(
                        self.enrolled_signature, signature
                    ) > self.reid_match_threshold:
                        # Fail closed if appearance cannot be verified or the
                        # tracker ID jumps to a different person after occlusion.
                        continue
                return tracklet

        if not candidates:
            return None

        if self.enrolled_signature is not None and self.reidentification_enabled:
            scored = [
                (appearance_distance(self.enrolled_signature, signature), tracklet)
                for tracklet, _distance, signature in candidates
                if signature is not None
            ]
            if not scored:
                self._set_block_reason("enrolled_subject_cannot_be_verified")
                return None
            score, target = min(scored, key=lambda item: item[0])
            if score > self.reid_match_threshold:
                self._set_block_reason("enrolled_subject_not_found")
                return None
            self.get_logger().info(
                "Re-identified enrolled subject (appearance distance %.3f)." % score
            )
        else:
            target = min(candidates, key=lambda item: item[1])[0]
        self.locked_target_id = target.id
        self.get_logger().info("Locked onto person track ID %s" % target.id)
        return target

    def _request_enrollment(self):
        if (
            self.visible_candidate_count == 1
            and self.last_frame is not None
            and self.last_target_bounds is not None
        ):
            if not self._target_is_stale():
                self._save_enrollment(self.last_target_bounds)
                return
        self.enrollment_pending = True
        self._publish_event("enrollment_waiting_for_visible_target")
        self.get_logger().info(
            "Enrollment requested; waiting for a visible, valid-depth person."
        )

    def _save_enrollment(self, bounds):
        signature = appearance_signature(self.last_frame, bounds)
        if signature is None:
            self._publish_event("enrollment_failed:invalid_person_crop")
            return
        payload = {
            "version": 1,
            "kind": "hsv_clothing_histogram",
            "bins": list(HISTOGRAM_BINS),
            "signature": signature.tolist(),
        }
        try:
            self.enrollment_file.parent.mkdir(parents=True, exist_ok=True)
            self.enrollment_file.write_text(json.dumps(payload))
            self.enrollment_file.chmod(0o600)
        except OSError as error:
            self._publish_event("enrollment_failed:file_write")
            self.get_logger().error("Could not save target enrollment: %s" % error)
            return
        self.enrolled_signature = signature
        self.enrollment_pending = False
        self._publish_event("target_enrolled")
        self.get_logger().info(
            "Enrolled the visible subject using a clothing-color signature."
        )

    def _clear_enrollment(self):
        if self.require_enrollment_for_motion and self.mode != "IDLE":
            self.set_mode("IDLE")
        self.enrolled_signature = None
        self.enrollment_pending = False
        try:
            self.enrollment_file.unlink(missing_ok=True)
        except OSError as error:
            self.get_logger().warning("Could not delete enrollment file: %s" % error)
        self._publish_event("target_enrollment_cleared")

    def _target_observation(self, tracklet):
        coordinates = getattr(tracklet, "spatialCoordinates", None)
        if coordinates is None:
            return None
        distance_m = float(getattr(coordinates, "z", 0.0)) / 1000.0
        if not self.minimum_valid_depth_m <= distance_m <= self.maximum_valid_depth_m:
            return None

        roi = tracklet.roi
        try:
            center_x = float(roi.x + roi.width / 2.0)
        except AttributeError:
            top_left = roi.topLeft()
            bottom_right = roi.bottomRight()
            center_x = float((top_left.x + bottom_right.x) / 2.0)

        if center_x > 1.0 and self.last_frame is not None:
            center_x /= float(self.last_frame.shape[1])
        return center_x, distance_m

    def _target_is_stale(self):
        if self.last_target_time is None:
            return True
        return time.monotonic() - self.last_target_time > self.target_timeout_sec

    def _publish_pointcloud(self, packet):
        points = np.asarray(packet.getPoints(), dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            return
        points = points[::self.pointcloud_decimation]
        finite = np.isfinite(points).all(axis=1)
        depth = points[:, 2]
        valid = finite & (
            (depth >= self.minimum_valid_depth_m)
            & (depth <= self.maximum_valid_depth_m)
        )
        points = points[valid]
        if points.size == 0:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "oakd_rgb_camera_optical_frame"
        self.pointcloud_publisher.publish(
            point_cloud2.create_cloud_xyz32(header, points.tolist())
        )

    def _control_robot(self):
        safety_reason = self._safety_block_reason()
        if safety_reason is not None:
            self.stop_robot()
            self._set_block_reason(safety_reason)
            return

        linear_x, angular_z = compute_motion(
            mode=self.mode,
            target_visible=True,
            center_x=self.smoothed_x,
            distance_m=self.smoothed_distance,
            target_distance_m=self.target_distance_m,
            distance_deadband_m=self.distance_deadband_m,
            center_deadband=self.center_deadband,
            follow_speed=self.follow_speed,
            turn_speed=self.turn_speed,
        )
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z

        if self.motion_enabled:
            self.cmd_vel_publisher.publish(command)
        now = time.monotonic()
        if now - self.last_motion_log_time >= 1.0:
            ranges = {
                name: (
                    None if not math.isfinite(value[0]) else round(value[0], 3)
                )
                for name, value in self.range_readings.items()
            }
            self.get_logger().info(
                "TRACK mode=%s target=%.2fm center=%.3f cmd=(%.3f,%.3f) "
                "lidar=%.3fm ranges=%s"
                % (
                    self.mode,
                    self.smoothed_distance,
                    self.smoothed_x,
                    command.linear.x,
                    command.angular.z,
                    self.front_lidar_distance,
                    ranges,
                )
            )
            self.last_motion_log_time = now
        self._set_block_reason(None)

    def scan_callback(self, message):
        front_ranges = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) > self.front_sector_radians:
                continue
            if not math.isfinite(distance):
                continue
            if message.range_min <= distance <= message.range_max:
                front_ranges.append(distance)

        self.front_lidar_distance = min(front_ranges) if front_ranges else math.inf
        self.last_lidar_time = time.monotonic()

    def ultrasonic_callback(self, message):
        self._store_range("ultrasonic", message.range)

    def ir_left_callback(self, message):
        self._store_range("ir_left", message.range)

    def ir_right_callback(self, message):
        self._store_range("ir_right", message.range)

    def _store_range(self, name, distance):
        self.range_readings[name] = (float(distance), time.monotonic())

    def bumper_callback(self, message):
        if message.width * message.height > 0:
            self.last_bumper_time = time.monotonic()

    def safety_stop_callback(self, message):
        self.last_safety_stop_time = time.monotonic()
        self.bumper_pressed = bool(message.data)

    def _poll_health(self):
        now = time.monotonic()
        if now - self.last_health_time < 1.0:
            return
        self.last_health_time = now
        try:
            self.pi_temp_c = float(
                Path("/sys/class/thermal/thermal_zone0/temp").read_text().strip()
            ) / 1000.0
        except (OSError, ValueError):
            self.pi_temp_c = None
        try:
            self.oak_temp_c = float(self.device.getChipTemperature().average)
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
                timeout=0.20,
            )
            self.throttled_value = int(
                result.stdout.strip().split("=", 1)[1], 16
            )
        except (OSError, ValueError, IndexError, subprocess.SubprocessError):
            self.throttled_value = None

    def _safety_block_reason(self):
        now = time.monotonic()
        if self.require_bumper:
            if self.last_safety_stop_time is None:
                return "waiting_for_bumper"
            if now - self.last_safety_stop_time > self.sensor_timeout_sec:
                return "bumper_stale"
        if self.bumper_pressed:
            return "bumper_pressed"
        if self.last_bumper_time is not None:
            if now - self.last_bumper_time <= self.bumper_hold_sec:
                return "bumper_pressed"

        if self.require_lidar:
            if self.last_lidar_time is None:
                return "waiting_for_lidar"
            if now - self.last_lidar_time > self.sensor_timeout_sec:
                return "lidar_stale"

        if self.front_lidar_distance < self.lidar_stop_distance_m:
            return "lidar_obstacle"

        required_ranges = []
        if self.require_ultrasonic:
            required_ranges.append("ultrasonic")
        if self.require_ir:
            required_ranges.extend(("ir_left", "ir_right"))
        for name in required_ranges:
            if name not in self.range_readings:
                return "waiting_for_%s" % name
            _distance, timestamp = self.range_readings[name]
            if now - timestamp > self.sensor_timeout_sec:
                return "%s_stale" % name

        ultrasonic = self.range_readings.get("ultrasonic")
        if ultrasonic is not None:
            distance, timestamp = ultrasonic
            if (
                now - timestamp <= self.sensor_timeout_sec
                and math.isfinite(distance)
                and distance < self.range_stop_distance_m
            ):
                return "ultrasonic_obstacle"

        ir_obstacles = []
        for name in ("ir_left", "ir_right"):
            reading = self.range_readings.get(name)
            if reading is None:
                continue
            distance, timestamp = reading
            if (
                now - timestamp <= self.sensor_timeout_sec
                and math.isfinite(distance)
                and distance < self.range_stop_distance_m
            ):
                ir_obstacles.append(name)
        if self.ir_stop_requires_both:
            if len(ir_obstacles) == 2:
                return "ir_obstacle"
        elif ir_obstacles:
            return "%s_obstacle" % ir_obstacles[0]

        if self.pi_temp_c is None or self.oak_temp_c is None:
            return "temperature_unavailable"
        if self.throttled_value is None:
            return "power_status_unavailable"
        current_bits = self.throttled_value & 0xF
        if current_bits & 0x1:
            return "pi_undervoltage"
        if current_bits & 0xE and not self.allow_hot_operation:
            return "pi_heat_throttling"
        if self.pi_temp_c >= self.absolute_stop_pi_temp_c:
            return "pi_absolute_over_temperature"
        if (
            not self.allow_hot_operation
            and self.pi_temp_c >= self.maximum_run_pi_temp_c
        ):
            return "pi_over_temperature"
        if self.oak_temp_c >= self.maximum_oak_temp_c:
            return "oak_over_temperature"
        return None

    def _set_block_reason(self, reason):
        if reason == self.last_block_reason:
            return
        self.last_block_reason = reason
        if reason is not None:
            self.get_logger().warning("Motion blocked: %s" % reason)

    def stop_robot(self):
        if not self.motion_enabled or not self.context.ok():
            return
        try:
            self.cmd_vel_publisher.publish(Twist())
        except RCLError:
            # ROS may invalidate the context before launch invokes node cleanup.
            pass

    def save_photo(self):
        self.recording_directory.mkdir(parents=True, exist_ok=True)
        path = self.recording_directory / (
            "photo_%s.jpg" % datetime.now().strftime("%Y%m%d_%H%M%S")
        )
        if cv2.imwrite(str(path), self.last_frame):
            self._publish_event("photo_saved:%s" % path)
            self.get_logger().info("Saved photo to %s" % path)
        else:
            self._publish_event("photo_failed")
            self.get_logger().error("Failed to save photo to %s" % path)
        self.photo_pending = False

    def start_recording(self):
        self.recording_requested = True
        if self.video_writer is not None:
            return
        if self.last_frame is None:
            self.get_logger().info("Recording will start when the first frame arrives.")
            return

        self._open_recording_writer()

    def _open_recording_writer(self):
        if self.video_writer is not None or self.last_frame is None:
            return

        self.recording_directory.mkdir(parents=True, exist_ok=True)
        free_bytes = shutil.disk_usage(str(self.recording_directory)).free
        free_megabytes = free_bytes / (1024.0 * 1024.0)
        if free_megabytes < self.minimum_free_disk_mb:
            self.get_logger().error(
                "Not enough free disk space to record (%.0f MB available)."
                % free_megabytes
            )
            self.recording_requested = False
            return
        self.recording_path = self.recording_directory / (
            "video_%s.mp4" % datetime.now().strftime("%Y%m%d_%H%M%S")
        )
        height, width = self.last_frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(
            str(self.recording_path), fourcc, self.recording_fps, (width, height)
        )
        if not writer.isOpened():
            self.get_logger().error("Could not open video writer.")
            writer.release()
            self.recording_path = None
            self.recording_requested = False
            return
        self.video_writer = writer
        self.recording_start_time = time.monotonic()
        self._publish_event("recording_started:%s" % self.recording_path)

    def stop_recording(self):
        self.recording_requested = False
        if self.video_writer is None:
            return
        self.video_writer.release()
        self.video_writer = None
        self.recording_start_time = None
        self._publish_event("recording_stopped:%s" % self.recording_path)
        self.get_logger().info("Saved video to %s" % self.recording_path)
        self.recording_path = None

    def _publish_event(self, event):
        if not self.context.ok():
            return
        try:
            self.event_publisher.publish(String(data=event))
        except RCLError:
            # Event delivery is best-effort while the process is shutting down.
            pass

    def _publish_status_periodically(self):
        now = time.monotonic()
        if now - self.last_status_time < 0.5:
            return
        self.last_status_time = now
        status = {
            "mode": self.mode.lower(),
            "target_id": self.locked_target_id,
            "target_visible": not self._target_is_stale(),
            "target_distance_m": round(self.smoothed_distance, 2),
            "target_center_x": round(self.smoothed_x, 3),
            "front_lidar_m": (
                round(self.front_lidar_distance, 2)
                if math.isfinite(self.front_lidar_distance)
                else None
            ),
            "motion_blocked_by": self.last_block_reason,
            "motion_enabled": self.motion_enabled,
            "vision_runtime": "depthai_2.32",
            "lidar_stop_distance_m": self.lidar_stop_distance_m,
            "range_stop_distance_m": self.range_stop_distance_m,
            "ir_stop_requires_both": self.ir_stop_requires_both,
            "absolute_stop_pi_temp_c": self.absolute_stop_pi_temp_c,
            "ultrasonic": self._status_range("ultrasonic"),
            "ir_left": self._status_range("ir_left"),
            "ir_right": self._status_range("ir_right"),
            "bumper_pressed": self.bumper_pressed,
            "pi_temp_c": self.pi_temp_c,
            "oak_temp_c": self.oak_temp_c,
            "hot_operation_override": self.allow_hot_operation,
            "throttled": (
                None
                if self.throttled_value is None
                else "0x%x" % self.throttled_value
            ),
            "recording": self.video_writer is not None,
            "subject_enrolled": self.enrolled_signature is not None,
            "enrollment_required": self.require_enrollment_for_motion,
            "visible_person_count": self.visible_candidate_count,
            "depth_obstacles_enabled": self.publish_pointcloud,
        }
        self.status_publisher.publish(String(data=json.dumps(status)))

    def _status_range(self, name):
        reading = self.range_readings.get(name)
        if reading is None:
            return None
        distance, timestamp = reading
        return {
            "distance_m": (
                None if not math.isfinite(distance) else round(distance, 3)
            ),
            "age_sec": round(time.monotonic() - timestamp, 3),
        }

    def destroy_node(self):
        self.get_logger().info("Stopping person tracker and OAK-D pipeline.")
        try:
            try:
                self.stop_robot()
            except Exception as error:
                self.get_logger().warning(
                    "Could not publish the final stop command: %s" % error
                )
            try:
                self.stop_recording()
            except Exception as error:
                self.get_logger().warning(
                    "Could not close the video recording: %s" % error
                )
            for queue_name in (
                "rgb_queue", "tracklets_queue", "pointcloud_queue"
            ):
                queue = getattr(self, queue_name, None)
                if queue is not None and hasattr(queue, "close"):
                    try:
                        queue.close()
                    except Exception as error:
                        self.get_logger().warning(
                            "Could not close %s: %s" % (queue_name, error)
                        )
            device = getattr(self, "device", None)
            if device is not None and hasattr(device, "close"):
                try:
                    device.close()
                    self.device = None
                except Exception as error:  # Ensure ROS/GPIO cleanup still runs.
                    self.get_logger().warning(
                        "Could not close OAK-D device cleanly: %s" % error
                    )
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PersonFollowerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
