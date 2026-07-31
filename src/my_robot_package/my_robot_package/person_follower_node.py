#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import json
import time
import numpy as np
import math
import depthai as dai
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from datetime import datetime

# A map for some common MobileNet-SSD labels
LABEL_MAP = {
    0: "person", 1: "bicycle", 2: "car", 3: "motorbike", 
    15: "cat", 16: "dog", 58: "pottedplant", 67: "cell phone"
}

def yaw_to_quat(yaw: float):
    """Converts a yaw angle to a quaternion."""
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class PersonFollowerNode(Node):
    TARGET_DISTANCE_M = 1.5  # Target distance to maintain from the person
    ORBIT_RADIUS_M = 2.0     # Radius for the orbit mode
    PERSON_LABEL = 0         # Label for 'person' from the LABEL_MAP
    MAX_FRAMES_LOST = 60     # Number of frames to wait before declaring target lost (e.g., 60 frames at 5Hz = 12 seconds)

    def __init__(self):
        super().__init__('person_follower_node')
        
        # --- State Machine & Target Tracking ---
        self.current_mode = 'IDLE'  # Modes: IDLE, FOLLOW_BEHIND, ORBIT, TAKE_PHOTO
        self.locked_target_id = None
        self.frames_lost = 0
        
        # --- TF2 for Coordinate Transformations ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # --- Publishers and Subscribers ---
        self.image_publisher = self.create_publisher(Image, '/oakd/color/image_raw', 10)
        self.event_publisher = self.create_publisher(StringMsg, 'assistant/event', 10)
        self.command_subscriber = self.create_subscription(StringMsg, 'assistant/command', self.command_callback, 10)
        
        self.cv_bridge = CvBridge()

        # --- Navigation Action Client ---
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.get_logger().info("Initializing OAK-D Person Follower for Nav2...")
        
        # --- OAK-D Pipeline Setup ---
        self.pipeline = self.create_pipeline()
        self.device = dai.Device(self.pipeline)
        self.qTracklets = self.device.getOutputQueue(name="tracklets", maxSize=4, blocking=False)
        self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        # --- Main Loop Timer ---
        self.timer = self.create_timer(0.5, self.timer_callback) # Poll at 2Hz
        self.get_logger().info("Person Follower Node is online. Waiting for commands.")

    def create_pipeline(self):
        pipeline = dai.Pipeline()
        camRgb = pipeline.create(dai.node.ColorCamera)
        detectionNetwork = pipeline.create(dai.node.MobileNetDetectionNetwork)
        objectTracker = pipeline.create(dai.node.ObjectTracker)
        
        # Configure RGB Camera
        camRgb.setPreviewSize(300, 300)
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        # Configure Detection Network
        detectionNetwork.setConfidenceThreshold(0.5)
        detectionNetwork.setBlobPath(blobconverter.from_zoo(name="mobilenet-ssd", shaves=6))
        
        # Configure Object Tracker
        objectTracker.setDetectionLabelsToTrack([self.PERSON_LABEL])
        objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.UNIQUE_ID)

        # Linking
        camRgb.preview.link(detectionNetwork.input)
        detectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)
        detectionNetwork.out.link(objectTracker.inputDetections)

        # Create outputs
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        camRgb.preview.link(xoutRgb.input)

        xoutTracker = pipeline.create(dai.node.XLinkOut)
        xoutTracker.setStreamName("tracklets")
        objectTracker.out.link(xoutTracker.input)
        
        return pipeline

    def command_callback(self, msg):
        self.get_logger().info(f"Received command: '{msg.data}'")
        command_parts = msg.data.split(':')
        command = command_parts[0]
        
        if command == 'start_follow':
            mode = command_parts[1] if len(command_parts) > 1 else 'behind'
            if mode == 'behind':
                self.current_mode = 'FOLLOW_BEHIND'
            elif mode == 'orbit':
                self.current_mode = 'ORBIT'
            self.locked_target_id = None
            self.frames_lost = 0
            self.get_logger().info(f"Mode set to {self.current_mode}")
        elif command == 'stop_follow':
            self.current_mode = 'IDLE'
            self.cancel_current_goal()
        elif command == 'take_photo':
            self.current_mode = 'TAKE_PHOTO'

    def cancel_current_goal(self):
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.get_logger().info('Cancelling current Nav2 goal.')
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None

    def timer_callback(self):
        in_rgb = self.qRgb.tryGet()
        in_track = self.qTracklets.tryGet()

        if in_rgb is not None:
            frame = in_rgb.getCvFrame()
            # self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(frame, "bgr8"))
        
        if self.current_mode == 'IDLE':
            return

        if in_track is None:
            return

        target = self.find_target(in_track.tracklets)

        if target is None:
            self.frames_lost += 1
            if self.frames_lost > self.MAX_FRAMES_LOST:
                self.get_logger().warn("Target lost. Stopping robot.")
                self.cancel_current_goal()
                self.current_mode = 'IDLE'
                self.locked_target_id = None
                self.event_publisher.publish(StringMsg(data="event:target_lost"))
            return
        
        self.frames_lost = 0

        if self.current_mode == 'TAKE_PHOTO':
            if 'frame' in locals():
                self.handle_take_photo(frame)
            self.current_mode = 'IDLE'
            return

        person_point_cam = PointStamped()
        person_point_cam.header.frame_id = "oakd_rgb_camera_optical_frame"
        person_point_cam.header.stamp = self.get_clock().now().to_msg()
        # Note: SpatialDetectionNetwork provides coordinates in millimeters
        person_point_cam.point.x = float(target.spatialCoordinates.x / 1000.0)
        person_point_cam.point.y = float(target.spatialCoordinates.y / 1000.0)
        person_point_cam.point.z = float(target.spatialCoordinates.z / 1000.0)

        try:
            person_point_map = self.tf_buffer.transform(person_point_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
            goal_pose = self.calculate_goal_pose(person_point_map)
            if goal_pose:
                self.send_nav2_goal(goal_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'TF transform error: {e}')

    def find_target(self, tracklets):
        # If we have a locked target, try to find it again
        if self.locked_target_id is not None:
            for t in tracklets:
                if t.id == self.locked_target_id and t.status == dai.Tracklet.TrackingStatus.TRACKED:
                    return t
        
        # If we lost the target or don't have one, find the closest new person
        closest_person = None
        min_z = float('inf')
        for t in tracklets:
            if t.label == self.PERSON_LABEL and t.status == dai.Tracklet.TrackingStatus.TRACKED:
                if 0 < t.spatialCoordinates.z < min_z:
                    min_z = t.spatialCoordinates.z
                    closest_person = t
        
        if closest_person:
            self.locked_target_id = closest_person.id
            self.get_logger().info(f"Target locked: ID {self.locked_target_id}")
        
        return closest_person

    def calculate_goal_pose(self, person_point_map):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        robot_pos = self.get_robot_position_in_map()
        if robot_pos is None: return None

        person_x = person_point_map.point.x
        person_y = person_point_map.point.y
        
        angle_to_person = math.atan2(person_y - robot_pos.y, person_x - robot_pos.x)

        if self.current_mode == 'FOLLOW_BEHIND':
            dist = self.TARGET_DISTANCE_M
            goal_pose.pose.position.x = person_x - dist * math.cos(angle_to_person)
            goal_pose.pose.position.y = person_y - dist * math.sin(angle_to_person)
            goal_pose.pose.orientation = yaw_to_quat(angle_to_person)
        
        elif self.current_mode == 'ORBIT':
            # Simplified orbit: go to a point 90 degrees to the right of the person, relative to the robot
            orbit_angle = angle_to_person - (math.pi / 2.0)
            goal_pose.pose.position.x = person_x + self.ORBIT_RADIUS_M * math.cos(orbit_angle)
            goal_pose.pose.position.y = person_y + self.ORBIT_RADIUS_M * math.sin(orbit_angle)
            # Face the person from the new orbit position
            angle_to_face = math.atan2(person_y - goal_pose.pose.position.y, person_x - goal_pose.pose.position.x)
            goal_pose.pose.orientation = yaw_to_quat(angle_to_face)
        
        else:
            return None
            
        return goal_pose

    def get_robot_position_in_map(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return trans.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get robot position in map frame: {e}')
            return None

    def send_nav2_goal(self, pose):
        if self.goal_handle and self.goal_handle.is_active:
            # Don't resend goals constantly, let Nav2 work.
            # A better implementation would check if the new goal is far from the old one.
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info('Sending new goal to Nav2.')
        self.nav_to_pose_client.wait_for_server(timeout_sec=2.0)
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2 server.')
            return
        self.get_logger().info('Goal accepted by Nav2 server.')

    def handle_take_photo(self, frame):
        if frame is not None:
            filename = f"photo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(filename, frame)
            self.get_logger().info(f"Photo saved as {filename}")
            self.event_publisher.publish(StringMsg(data=f"event:photo_taken:{filename}"))
        else:
            self.get_logger().warn("Could not take photo, no frame available.")
            self.event_publisher.publish(StringMsg(data="event:photo_failed"))

    def destroy_node(self):
        self.get_logger().info("Shutting down Person Follower.")
        self.cancel_current_goal()
        self.device.close()
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
