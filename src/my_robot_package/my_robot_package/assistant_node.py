#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import speech_recognition as sr
from openai import OpenAI
from std_msgs.msg import String as StringMsg
from pathlib import Path
from cv_bridge import CvBridge
import os
import json
from ddgs import DDGS
import sys
import threading
from contextlib import contextmanager
from datetime import datetime
import cv2
import base64
import subprocess
import time

@contextmanager
def suppress_c_warnings():
    """Temporarily redirect OS-level stderr to /dev/null to hide ALSA and JACK spam."""
    sys.stderr.flush()
    null_fd = os.open(os.devnull, os.O_RDWR)
    save_fd = os.dup(2)
    os.dup2(null_fd, 2)
    try:
        yield
    finally:
        sys.stderr.flush()
        os.dup2(save_fd, 2)
        os.close(null_fd)
        os.close(save_fd)

class AssistantNode(Node):
    def __init__(self):
        super().__init__('assistant_node')
        
        # Initialize OpenAI (Requires OPENAI_API_KEY environment variable)
        self.client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
        self.recognizer = sr.Recognizer()
        self.cv_bridge = CvBridge()

        # --- PERFORMANCE TUNING ---
        # Lower threshold makes the mic more sensitive to catch the wake word
        self.recognizer.energy_threshold = 1000
        # Shorter pause threshold for faster response after user stops talking
        self.recognizer.pause_threshold = 1.5
        
        # Initialize chat history with a smarter system prompt
        self.chat_history = [
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
 from tf2_geometry_msgs.py import do_transform_point
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
         self.timer = self.create_timer(0.2, self.timer_callback) # Poll at 5Hz
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
             self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(frame, "bgr8"))
         
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
 from tf2_geometry_msgs.py import do_transform_point
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
         self.timer = self.create_timer(0.2, self.timer_callback) # Poll at 5Hz
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
             self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(frame, "bgr8"))
         
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
 from tf2_geometry_msgs.py import do_transform_point
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
         self.timer = self.create_timer(0.2, self.timer_callback) # Poll at 5Hz
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
             self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(frame, "bgr8"))
         
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
            {"role": "system", "content": "You are Bob, a helpful robot assistant. To answer questions about your physical environment, you MUST use the `analyze_visual_scene` tool. For real-time info (weather, news), use `search_web`. To control your actions like following a person, orbiting, or taking a photo, you MUST use the `set_robot_mode` tool. For general system tasks, use `execute_bash_command`. For all other questions, answer from your training data. Keep answers brief (1-2 sentences) and speak in plain English for the text-to-speech engine."}
        ] 
        
        # --- VISION: Subscribe to the raw camera image feed and detections ---
        self.create_subscription(
            Image,
            '/oakd/color/image_raw',
            self.image_callback,
            10
        )
        self.latest_frame = None
        self.last_frame_time = 0
        self.image_lock = threading.Lock()
        self.get_logger().info("Subscribed to /oakd/color/image_raw for VLM analysis.")

        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "search_web",
                    "description": "Search the internet for real-time information, news, weather, or live sports scores.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "The search query to use."}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "execute_bash_command",
                    "description": "Execute a bash command on the robot's Linux system. Use this for general system tasks, file operations, or running scripts.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "command": {"type": "string", "description": "The bash command to run. Remember to use '&' for long-running launch files."}
                        },
                        "required": ["command"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "set_robot_mode",
                    "description": "Set the robot's operational mode. Use this to start or stop following a person, or to perform special actions like orbiting or taking a photo.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "mode": {
                                "type": "string", 
                                "description": "The desired mode.",
                                "enum": ["follow_behind", "orbit", "take_photo", "stop"]
                            }
                        },
                        "required": ["mode"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "analyze_visual_scene",
                    "description": "Analyzes the robot's camera view to answer a specific question about the environment. Use this for any visual questions like 'What do you see?', 'Describe the room.', 'What kind of plant is that?', or 'Is there a person here?'.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "question": {"type": "string", "description": "The specific question to answer about the scene."}
                        },
                        "required": ["question"]
                    }
                }
            }
        ]
        
        # Publisher for sending commands to the follower node
        self.command_publisher = self.create_publisher(
            StringMsg,
            '/assistant/command',
            10)
        
        # Start the listening loop in a background thread
        self.listen_thread = threading.Thread(target=self.listening_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()
        
    # --- VISION: Callback to store the latest camera frame ---
    def image_callback(self, msg):
        with self.image_lock:
            self.latest_frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_frame_time = time.time()

    # --- VISION: Function that the AI tool will call ---
    def analyze_visual_scene(self, question):
        self.get_logger().info(f"Analyzing scene with question: '{question}'")
        with self.image_lock:
            if self.latest_frame is None or (time.time() - self.last_frame_time) > 3.0:
                return "I'm not receiving a camera feed right now. Please check if the camera node is running."

            # Encode the image to base64
            _, buffer = cv2.imencode('.jpg', self.latest_frame)
            base64_image = base64.b64encode(buffer).decode('utf-8')

        try:
            # Call OpenAI's Vision API
            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": "You are a highly intelligent robotic vision system. Analyze the image to answer the user's question. If asked to identify objects, provide specific details like exact brand names, models, or plant species. If there is foreign text in the image, you must translate it to English. Keep answers brief (1-2 sentences) and conversational."},
                    {"role": "user", "content": [
                        {"type": "text", "text": question},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}", "detail": "low"}}
                    ]}
                ],
                max_tokens=150
            )
            description = response.choices[0].message.content
            self.get_logger().info(f"VLM Analysis: {description}")
            return description
        except Exception as e:
            self.get_logger().error(f"Error calling VLM API: {e}")
            return "I'm having trouble analyzing the image right now."

    def listening_loop(self):
        try:
            # Wrap the microphone initialization and opening in the C-warning suppressor
            with suppress_c_warnings():
                # By passing None, PyAudio uses the system default (PulseAudio/Pipewire)
                # which safely manages shared access to the hardware microphone.
                mic = sr.Microphone(device_index=None)
                source = mic.__enter__()
                
            self.get_logger().info("Microphone connected via System Default (Pulse/Pipewire).")
                
            if source.stream is None:
                raise OSError("PyAudio failed to open the default audio stream. The device might be busy, muted, or disconnected.")
        except Exception as e:
            self.get_logger().error(f"Microphone init failed! Error: {e}")
            return
            
        try:
            self.get_logger().info("Bob Online. Waiting for wake word 'Hi Bob'...")
            # Calibrate briefly for background noise
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            
            while rclpy.ok():
                try:
                    # Listen in shorter bursts for the wake word for better responsiveness
                    audio = self.recognizer.listen(source, timeout=0.5, phrase_time_limit=3)
                    text = self.recognizer.recognize_google(audio).lower()
                    
                    # Phonetic variations in case the STT engine misinterprets "Bob"
                    wake_words = ["hi bob", "hey bob", "high bob", "hello bob"]
                    if any(w in text for w in wake_words):
                        self.get_logger().info(f"Wake word detected! (Heard: '{text}')")
                        os.system("espeak-ng 'Yes?'") # Use fast, local TTS for acknowledgment
                        
                        # Enter an active conversation loop
                        conversation_active = True
                        while conversation_active and rclpy.ok():
                            self.get_logger().info("Listening for command...")
                            try:
                                # Wait up to 8 seconds for the user to reply before ending the active conversation
                                audio_cmd = self.recognizer.listen(source, timeout=8, phrase_time_limit=25)
                                cmd_text = self.recognizer.recognize_google(audio_cmd)
                                self.get_logger().info(f"You said: {cmd_text}")
                                
                                self.process_and_respond(cmd_text)
                            except sr.WaitTimeoutError:
                                self.get_logger().info("Conversation ended due to silence.")
                                conversation_active = False
                            except sr.UnknownValueError:
                                # End active conversation if we hear indistinguishable noise
                                conversation_active = False
                        self.get_logger().info("Waiting for wake word 'Hi Bob'...")
                    # No 'else' block to avoid spamming logs with background noise
                        
                except sr.WaitTimeoutError:
                    pass # Normal timeout while waiting for wake word, loop again
                except sr.UnknownValueError:
                    pass # Background noise, ignore
                except Exception as e:
                    self.get_logger().error(f"Audio Error: {e}")
        finally:
            if getattr(source, 'stream', None) is not None:
                mic.__exit__(None, None, None)

    def process_and_respond(self, text):
        try:
            # Update the system prompt with the current exact date and time
            now = datetime.now().strftime("%A, %B %d, %Y %I:%M %p")
            self.chat_history[0]["content"] = f"You are Bob, a helpful robot assistant. To answer questions about your physical environment, you MUST use the `analyze_visual_scene` tool. For real-time info (weather, news), use `search_web`. To control your actions like following a person, orbiting, or taking a photo, you MUST use the `set_robot_mode` tool. For general system tasks, use `execute_bash_command`. For all other questions, answer from your training data. Keep answers brief (1-2 sentences) and speak in plain English for the text-to-speech engine. The current local date and time is {now}."

            # Add user input to history
            self.chat_history.append({"role": "user", "content": text})
            
            # Keep history manageable: System prompt + 30 recent messages
            # (Increased because web searches require saving extra messages)
            if len(self.chat_history) > 31:
                self.chat_history = [self.chat_history[0]] + self.chat_history[-30:]

            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=self.chat_history,
                tools=self.tools,
                tool_choice="auto"
            )
            
            message = response.choices[0].message
            
            # Did the AI decide it needs to search the web?
            if message.tool_calls:
                self.get_logger().info("AI is using a tool...")
                self.chat_history.append(message)  # Add the AI's tool request to history
                
                for tool_call in message.tool_calls:
                    if tool_call.function.name == "search_web":
                        args = json.loads(tool_call.function.arguments)
                        self.get_logger().info(f"[DuckDuckGo Searching: {args['query']}]")
                        try:
                            results = DDGS().text(args["query"], max_results=3)
                            search_result = json.dumps(results)
                        except Exception as e:
                            search_result = f"Search failed: {e}"
                        
                        self.chat_history.append({
                            "role": "tool",
                            "tool_call_id": tool_call.id,
                            "content": search_result
                        })
                    elif tool_call.function.name == "execute_bash_command":
                        args = json.loads(tool_call.function.arguments)
                        cmd = args.get('command', '')
                        self.get_logger().info(f"[Executing Bash: {cmd}]")
                        try:
                            # Run the command with a 10s timeout so the AI doesn't hang forever
                            result = subprocess.run(cmd, shell=True, text=True, capture_output=True, timeout=10)
                            output = result.stdout if result.stdout else result.stderr
                            if not output:
                                output = "Command executed successfully with no output."
                            output = output[:2000] # Truncate massive logs
                        except subprocess.TimeoutExpired:
                            output = "Command timed out after 10 seconds. (If this was a launch file, it is running in the background)."
                        except Exception as e:
                            output = f"Command failed: {e}"
                        
                        self.chat_history.append({"role": "tool", "tool_call_id": tool_call.id, "content": output})
                    
                    elif tool_call.function.name == "set_robot_mode":
                        args = json.loads(tool_call.function.arguments)
                        mode = args.get('mode', 'stop')
                        self.get_logger().info(f"[Setting Robot Mode: {mode}]")
                        command_str = ""
                        if mode == "stop":
                            command_str = "stop_follow"
                        else:
                            command_str = f"start_follow:{mode}"
                        
                        self.command_publisher.publish(StringMsg(data=command_str))
                        tool_output = f"Command '{mode}' sent to the robot's motion controller."
                        self.chat_history.append({"role": "tool", "tool_call_id": tool_call.id, "content": tool_output})

                    elif tool_call.function.name == "analyze_visual_scene":
                        args = json.loads(tool_call.function.arguments)
                        question = args.get('question', 'What do you see?')
                        camera_data = self.analyze_visual_scene(question=question)
                        self.chat_history.append({
                            "role": "tool",
                            "tool_call_id": tool_call.id,
                            "content": camera_data
                        })
                
                # Call OpenAI a second time, this time with the search results included!
                response = self.client.chat.completions.create(
                    model="gpt-4o",
                    messages=self.chat_history
                )
                message = response.choices[0].message

            reply = message.content
            self.get_logger().info(f"Robot says: {reply}")
            
            # Add the AI's response to history so it remembers the conversation
            self.chat_history.append({"role": "assistant", "content": reply})

            # --- UPGRADE: Use OpenAI's high-quality TTS for a natural voice ---
            self.speak(reply)
            
        except Exception as e:
            self.get_logger().error(f"AI/Network Error: {e}")

    def speak(self, text):
        try:
            # It requires an internet connection and will incur small API costs.
            # NOTE: You may need to install an MP3 player: sudo apt-get install mpg123
            speech_file_path = Path("/tmp/assistant_reply.mp3")
            with self.client.audio.speech.with_streaming_response.create(
                model="tts-1",
                voice="alloy", # Other voices: echo, fable, onyx, nova, shimmer
                input=text
            ) as response:
                response.stream_to_file(speech_file_path)
            
            # Play the generated audio file
            os.system(f"mpg123 {speech_file_path}")

            
        except Exception as e:
            self.get_logger().error(f"AI/Network Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AssistantNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()