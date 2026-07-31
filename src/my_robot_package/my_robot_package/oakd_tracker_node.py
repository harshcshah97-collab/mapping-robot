#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import Image 
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge          
import cv2                              
import json
import time
import numpy as np
import math

# --- START DIAGNOSTIC BLOCK ---
try:
    import depthai as dai
    print(f"Successfully imported depthai version: {dai.__version__}")
except (ImportError, AttributeError) as e:
    print("\n\nFATAL ERROR: Failed to import depthai.")
    print(f"Error details: {e}")
    print("Please ensure the depthai library is installed correctly (`pip install depthai`).\n\n")
    exit(1)
# --- END DIAGNOSTIC BLOCK ---

class OakdTrackerNode(Node):
    PERSON_LABEL = 0 
    TARGET_DISTANCE_M = 0.5  
    LINEAR_SPEED_CAP = 0.3   
    ANGULAR_SPEED_CAP = 1.0  
    
    LABEL_MAP = {
        0: "person", 1: "bicycle", 2: "car", 3: "motorbike", 
        15: "cat", 16: "dog", 58: "pottedplant", 67: "cell phone"
    }
    
    def __init__(self):
        super().__init__('oakd_tracker_node')
        
        self.is_following = False
        
        # Algorithm Memory & Identity Tracking
        self.locked_target_id = None
        self.smoothed_x = 0.5 
        self.smoothed_z = self.TARGET_DISTANCE_M 
        self.alpha = 0.3 
        self.frames_lost = 0 
        self.MAX_FRAMES_LOST = 10 
        
        # --- ODOMETRY STATE ---
        self.current_yaw = 0.0
        self.target_yaw = None
        self.is_turning = False # Controls Saccadic (Stop-and-Look) state
        self.last_turn_time = 0.0 # NEW: Cooldown timer to prevent twitching
        
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.detections_publisher_ = self.create_publisher(StringMsg, 'oakd/detections_info', 10)
        self.image_publisher_ = self.create_publisher(Image, '/oakd/color/image_raw', 10)
        self.event_publisher_ = self.create_publisher(StringMsg, 'assistant/event', 10)
        
        self.cv_bridge = CvBridge()

        self.create_subscription(StringMsg, 'assistant/command', self.command_callback, 10)
        
        # Subscribe to your beautiful Odometry node
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.get_logger().info("Initializing OAK-D Fusion: Saccadic Odometry Tracking...")
        
        self.pipeline = dai.Pipeline()
        camRgb = self.pipeline.create(dai.node.Camera).build()
        monoLeft = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        
        leftOut = monoLeft.requestOutput((640, 400))
        rightOut = monoRight.requestOutput((640, 400))
        leftOut.link(stereo.left)
        rightOut.link(stereo.right)
        
        rgbOut = camRgb.requestOutput((640, 480), type=dai.ImgFrame.Type.BGR888i)
        
        model_description = dai.NNModelDescription("luxonis/yolov6-nano:r2-coco-512x288")
        spatialDetectionNetwork = self.pipeline.create(dai.node.SpatialDetectionNetwork).build(camRgb, stereo, model_description)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)
        
        spatialDetectionNetwork.setConfidenceThreshold(0.5)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.3)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000) 
        
        self.objectTracker = self.pipeline.create(dai.node.ObjectTracker)
        self.objectTracker.setDetectionLabelsToTrack([self.PERSON_LABEL])
        self.objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.UNIQUE_ID)
        self.objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        
        spatialDetectionNetwork.out.link(self.objectTracker.inputDetections)
        
        tracker_manip = self.pipeline.create(dai.node.ImageManip)
        tracker_manip.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
        spatialDetectionNetwork.passthrough.link(tracker_manip.inputImage)
        
        tracker_manip.out.link(self.objectTracker.inputTrackerFrame)
        tracker_manip.out.link(self.objectTracker.inputDetectionFrame)

        self.qTracklets = self.objectTracker.out.createOutputQueue(maxSize=4, blocking=False)
        self.qRgb = rgbOut.createOutputQueue(maxSize=4, blocking=False)
        self.qDepth = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

        self.get_logger().info("Starting V3 Pipeline on Device...")
        
        self.device = self.pipeline.start()
        
        self.timer = self.create_timer(0.1, self.timer_callback) 
        self.get_logger().info("OAK-D Tracker Online. Waiting for 'start_follow' command.")

    # Convert the incoming Quaternion from your encoder_odom_node into radians (Yaw)
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def command_callback(self, msg):
        if msg.data == 'start_follow':
            if not self.is_following:
                self.get_logger().info("Follow-Me mode ACTIVATED.")
                self.is_following = True
                self.locked_target_id = None 
                self.frames_lost = 0
                self.target_yaw = None
                self.is_turning = False
                self.last_turn_time = time.time()
        elif msg.data == 'stop_follow':
            if self.is_following:
                self.get_logger().info("Follow-Me mode DEACTIVATED.")
                self.is_following = False
                self.locked_target_id = None
                self.target_yaw = None
                self.is_turning = False
                self.cmd_vel_publisher_.publish(Twist())
                
    def timer_callback(self):
        latest_rgb = None
        while True:
            pkt = self.qRgb.tryGet()
            if pkt is None: break
            latest_rgb = pkt
            
        if latest_rgb is not None:
            frame = latest_rgb.getCvFrame()
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_publisher_.publish(img_msg)

        latest_depth = None
        while True:
            pkt = self.qDepth.tryGet()
            if pkt is None: break
            latest_depth = pkt

        latest_tracklets = None
        while True:
            pkt = self.qTracklets.tryGet()
            if pkt is None: break
            latest_tracklets = pkt
            
        cmd_vel_msg = Twist()
        
        # --- PROCESS YOLO CAMERA ---
        if latest_tracklets is not None:
            target = None
            detected_objects = []
            
            for t in latest_tracklets.tracklets:
                if t.spatialCoordinates.z > 100: 
                    label_name = self.LABEL_MAP.get(t.label, f"Object {t.label}")
                    detected_objects.append({
                        "label": f"{label_name} (ID: {t.id})",
                        "distance_m": round(t.spatialCoordinates.z / 1000.0, 2)
                    })
                        
            if self.is_following:
                if self.locked_target_id is None:
                    min_z = 9999
                    for t in latest_tracklets.tracklets:
                        if t.status.name != "LOST" and 100 < t.spatialCoordinates.z < min_z:
                            min_z = t.spatialCoordinates.z
                            target = t
                    if target:
                        self.locked_target_id = target.id
                        self.get_logger().info(f"Target Locked! Following ID: {self.locked_target_id}")
                else:
                    for t in latest_tracklets.tracklets:
                        if t.id == self.locked_target_id:
                            target = t
                            break

                if target and target.status.name != "LOST" and target.spatialCoordinates.z > 100:
                    self.frames_lost = 0
                    
                    raw_2d_center_x = target.roi.x + (target.roi.width / 2.0)
                    raw_z = target.spatialCoordinates.z / 1000.0
                    
                    # ANTI-TWITCH FIX 1: Freeze visual updates while physically turning!
                    # This prevents the camera's motion blur from corrupting the math.
                    if not self.is_turning:
                        self.smoothed_x = (0.7 * raw_2d_center_x) + (0.3 * self.smoothed_x)
                        self.smoothed_z = (self.alpha * raw_z) + ((1.0 - self.alpha) * self.smoothed_z)
                else:
                    self.frames_lost += 1
                    if self.frames_lost > 2:
                        min_z = 2500 
                        new_target = None
                        for t in latest_tracklets.tracklets:
                            if t.status.name != "LOST" and 100 < t.spatialCoordinates.z < min_z:
                                min_z = t.spatialCoordinates.z
                                new_target = t
                        if new_target:
                            self.locked_target_id = new_target.id
                            target = new_target
                            self.frames_lost = 0
                            self.get_logger().info(f"Target Re-acquired! New ID: {self.locked_target_id}")

            self.detections_publisher_.publish(StringMsg(data=json.dumps(detected_objects)))
        else:
            if self.is_following:
                self.frames_lost += 1

        # --- FUSE SACCADIC PHYSICS ---
        if self.is_following:
            if self.frames_lost < self.MAX_FRAMES_LOST:
                
                # --- PHASE 1: "LOOK AND LOCK" ---
                if not self.is_turning:
                    screen_error = self.smoothed_x - 0.5
                    
                    # ANTI-TWITCH FIX 2: Added a 1.5-second COOLDOWN timer after every turn!
                    current_time = time.time()
                    if abs(screen_error) > 0.15 and (current_time - self.last_turn_time) > 1.5:
                        # 1.22 is the ~70 degree Field of View of the OAK-D camera mapped to radians
                        angle_offset = -screen_error * 1.22 
                        
                        # ANTI-TWITCH FIX 3: Minimum angle threshold. 
                        # Ignore micro-adjustments under ~5.5 degrees (0.10 rad) entirely.
                        if abs(angle_offset) > 0.10:
                            # Lock in the physical world compass heading we want to snap to
                            self.target_yaw = self.current_yaw + angle_offset
                            self.is_turning = True 
                            self.get_logger().info(f"Target Acquired. Snapping odometry by {angle_offset:.2f} radians.")
                    else:
                        angular_speed = 0.0
                        self.target_yaw = None

                # --- PHASE 2: "BLIND TURN" (Pure Odometry Execution) ---
                if self.is_turning and self.target_yaw is not None:
                    # Calculate shortest path to the locked physical angle
                    yaw_error = math.atan2(math.sin(self.target_yaw - self.current_yaw), math.cos(self.target_yaw - self.current_yaw))
                    
                    # Odometry deadband (~3.5 degrees)
                    if abs(yaw_error) > 0.06:
                        # P-Gain: Turn aggressively since Odometry guarantees a perfect stop
                        base_turn = yaw_error * 2.0 
                        
                        # Minimum friction voltage 
                        if base_turn > 0:
                            angular_speed = max(base_turn, 0.22)
                        else:
                            angular_speed = min(base_turn, -0.22)
                            
                        # Hard limit max speed for safety
                        angular_speed = max(min(angular_speed, 0.50), -0.50)
                    else:
                        # WE HIT THE TARGET! Instantly cut power and switch back to "Looking" mode.
                        angular_speed = 0.0 
                        self.is_turning = False 
                        
                        # ANTI-TWITCH FIX 4: Start the cooldown and wipe the stale memory!
                        self.last_turn_time = time.time() 
                        self.smoothed_x = 0.5 
                elif not self.is_turning:
                    angular_speed = 0.0

                # Linear Distance 
                if self.smoothed_z > 1.5:
                    linear_speed = (self.smoothed_z - 1.5) * 0.3 
                else:
                    linear_speed = 0.0 
                    
                cmd_vel_msg.linear.x = float(max(min(linear_speed, self.LINEAR_SPEED_CAP), 0.0))
                cmd_vel_msg.angular.z = float(max(min(angular_speed, self.ANGULAR_SPEED_CAP), -self.ANGULAR_SPEED_CAP))
            else:
                cmd_vel_msg = Twist()
                self.smoothed_x = 0.5 
                self.smoothed_z = self.TARGET_DISTANCE_M
                self.target_yaw = None
                self.is_turning = False
                if self.locked_target_id is not None:
                    self.get_logger().warning("Lost visual of target entirely! Stopping.")
                    msg = StringMsg()
                    msg.data = "target_lost"
                    self.event_publisher_.publish(msg)
                    self.locked_target_id = None 
                    
            self.cmd_vel_publisher_.publish(cmd_vel_msg)
        else:
            self.cmd_vel_publisher_.publish(Twist())

    def destroy_node(self):
        self.get_logger().info("Shutting down OAK-D and freeing the USB port...")
        if hasattr(self, 'qTracklets'): self.qTracklets.close(); del self.qTracklets
        if hasattr(self, 'qRgb'): self.qRgb.close(); del self.qRgb
        if hasattr(self, 'qDepth'): self.qDepth.close(); del self.qDepth
        if hasattr(self, 'device') and self.device is not None:
            try: self.device.close()
            except Exception: pass
            del self.device
        if hasattr(self, 'pipeline'):
            try:
                if hasattr(self.pipeline, 'stop'): self.pipeline.stop()
                if hasattr(self.pipeline, 'close'): self.pipeline.close()
            except Exception: pass
            del self.pipeline
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OakdTrackerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        del node 
        rclpy.shutdown()

if __name__ == '__main__':
    main()