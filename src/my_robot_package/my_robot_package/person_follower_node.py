#!/usr/bin/env python3
"""OAK-D person tracking, safe following, framing, and recording."""

import json
import math
import shutil
import time
from datetime import datetime
from pathlib import Path

import cv2
import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

from my_robot_package.appearance_logic import (
    HISTOGRAM_BINS,
    appearance_distance,
    appearance_signature,
    roi_bounds,
)
from my_robot_package.tracking_logic import compute_motion


class PersonFollowerNode(Node):
    """Track one person with OAK-D spatial detections and command the base safely."""

    PERSON_LABEL = 0
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

        self.cv_bridge = CvBridge()
        self.get_logger().info("Initializing OAK-D v3 spatial person tracker...")
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
        self.declare_parameter(
            "model", "luxonis/yolov6-nano:r2-coco-512x288"
        )
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
        self.declare_parameter("motion_enabled", True)
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
        self.declare_parameter("reid_match_threshold", 0.35)
        self.declare_parameter(
            "enrollment_file", "~/.config/mapping-robot/target_enrollment.json"
        )

    def _load_parameters(self):
        self.initial_mode = str(self.get_parameter("initial_mode").value)
        self.model = str(self.get_parameter("model").value)
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
        self.motion_enabled = bool(self.get_parameter("motion_enabled").value)
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
        self.reid_match_threshold = float(
            self.get_parameter("reid_match_threshold").value
        )
        self.enrollment_file = Path(
            str(self.get_parameter("enrollment_file").value)
        ).expanduser()
        if not 0.0 <= self.reid_match_threshold <= 1.0:
            raise ValueError("reid_match_threshold must be between 0 and 1")

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
        """Build the DepthAI v3 RGB + stereo + spatial tracking pipeline."""
        self.pipeline = dai.Pipeline()

        rgb_camera = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A
        )
        mono_left = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B
        )
        mono_right = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C
        )

        stereo = self.pipeline.create(dai.node.StereoDepth)
        mono_left.requestOutput((640, 400)).link(stereo.left)
        mono_right.requestOutput((640, 400)).link(stereo.right)

        preview_output = rgb_camera.requestOutput(
            (640, 480), type=dai.ImgFrame.Type.BGR888i
        )
        model_description = dai.NNModelDescription(self.model)
        detector = self.pipeline.create(dai.node.SpatialDetectionNetwork).build(
            rgb_camera, stereo, model_description
        )
        detector.setConfidenceThreshold(0.55)
        detector.input.setBlocking(False)
        detector.setBoundingBoxScaleFactor(0.35)
        detector.setDepthLowerThreshold(int(self.minimum_valid_depth_m * 1000.0))
        detector.setDepthUpperThreshold(int(self.maximum_valid_depth_m * 1000.0))

        tracker = self.pipeline.create(dai.node.ObjectTracker)
        tracker.setDetectionLabelsToTrack([self.PERSON_LABEL])
        tracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
        tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.UNIQUE_ID)
        detector.passthrough.link(tracker.inputTrackerFrame)
        detector.passthrough.link(tracker.inputDetectionFrame)
        detector.out.link(tracker.inputDetections)

        self.rgb_queue = preview_output.createOutputQueue(maxSize=3, blocking=False)
        self.tracklets_queue = tracker.out.createOutputQueue(maxSize=3, blocking=False)
        self.pointcloud_queue = None
        if self.publish_pointcloud:
            pointcloud = self.pipeline.create(dai.node.PointCloud)
            pointcloud.initialConfig.setLengthUnit(dai.LengthUnit.METER)
            pointcloud.initialConfig.setTargetCoordinateSystem(
                dai.CameraBoardSocket.CAM_A
            )
            stereo.depth.link(pointcloud.inputDepth)
            self.pointcloud_queue = pointcloud.outputPointCloud.createOutputQueue(
                maxSize=2, blocking=False
            )
        self.pipeline.start()

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

        for tracklet, _distance, signature in candidates:
            if self.locked_target_id is not None and tracklet.id == self.locked_target_id:
                if self.enrolled_signature is not None and signature is not None:
                    if appearance_distance(
                        self.enrolled_signature, signature
                    ) > self.reid_match_threshold:
                        # Fail closed if the tracker ID appears to have jumped to
                        # a visibly different person after an occlusion.
                        continue
                return tracklet

        if not candidates:
            return None

        if self.enrolled_signature is not None and self.reidentification_enabled:
            scored = [
                (appearance_distance(self.enrolled_signature, signature), tracklet)
                for tracklet, _distance, signature in candidates
            ]
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
        if self.last_frame is not None and self.last_target_bounds is not None:
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

    def _safety_block_reason(self):
        now = time.monotonic()
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

        for name, reading in self.range_readings.items():
            distance, timestamp = reading
            if now - timestamp > self.sensor_timeout_sec:
                continue
            if math.isfinite(distance) and distance < self.range_stop_distance_m:
                return "%s_obstacle" % name
        return None

    def _set_block_reason(self, reason):
        if reason == self.last_block_reason:
            return
        self.last_block_reason = reason
        if reason is not None:
            self.get_logger().warning("Motion blocked: %s" % reason)

    def stop_robot(self):
        if self.motion_enabled:
            self.cmd_vel_publisher.publish(Twist())

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
        self.event_publisher.publish(String(data=event))

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
            "recording": self.video_writer is not None,
            "subject_enrolled": self.enrolled_signature is not None,
            "depth_obstacles_enabled": self.publish_pointcloud,
        }
        self.status_publisher.publish(String(data=json.dumps(status)))

    def destroy_node(self):
        self.get_logger().info("Stopping person tracker and OAK-D pipeline.")
        self.stop_robot()
        self.stop_recording()
        for queue_name in ("rgb_queue", "tracklets_queue", "pointcloud_queue"):
            queue = getattr(self, queue_name, None)
            if queue is not None and hasattr(queue, "close"):
                queue.close()
        if hasattr(self, "pipeline"):
            if hasattr(self.pipeline, "stop"):
                self.pipeline.stop()
            if hasattr(self.pipeline, "wait"):
                self.pipeline.wait()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
