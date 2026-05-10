#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from gpiozero import RotaryEncoder


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__("encoder_odom_node")

        # --- Parameters ---
        self.declare_parameter("left_a", 13)
        self.declare_parameter("left_b", 19)
        self.declare_parameter("right_a", 5)
        self.declare_parameter("right_b", 6)

        self.declare_parameter("ticks_per_rev", 494.0)      # your measured value
        self.declare_parameter("wheel_diameter", 0.060)     # meters (65mm)
        self.declare_parameter("wheel_separation", 0.240)   # meters (240mm)

        # Direction multipliers (+1 or -1)
        self.declare_parameter("left_dir", -1)   # left forward gave negative -> invert
        self.declare_parameter("right_dir", 1)   # right forward gave positive -> keep

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)

        self.declare_parameter("rate_hz", 30.0)

        # --- Load params ---
        self.left_a = int(self.get_parameter("left_a").value)
        self.left_b = int(self.get_parameter("left_b").value)
        self.right_a = int(self.get_parameter("right_a").value)
        self.right_b = int(self.get_parameter("right_b").value)

        self.ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)
        self.wheel_diameter = float(self.get_parameter("wheel_diameter").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)

        self.left_dir = int(self.get_parameter("left_dir").value)
        self.right_dir = int(self.get_parameter("right_dir").value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / max(self.rate_hz, 1.0)

        # --- Encoder objects (same as your test) ---
        self.left_enc = RotaryEncoder(a=self.left_a, b=self.left_b, max_steps=0)
        self.right_enc = RotaryEncoder(a=self.right_a, b=self.right_b, max_steps=0)

        self.last_left_steps = self.left_enc.steps
        self.last_right_steps = self.right_enc.steps

        # --- Odometry state ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = time.time()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info(
            f"Encoder odom started. L({self.left_a},{self.left_b}) R({self.right_a},{self.right_b}), "
            f"TPR={self.ticks_per_rev}, wheel_diam={self.wheel_diameter}, sep={self.wheel_separation}, "
            f"dirs L={self.left_dir} R={self.right_dir}"
        )

    def update(self):
        now = time.time()
        dt = max(now - self.last_time, 1e-6)
        self.last_time = now

        left_steps = self.left_enc.steps
        right_steps = self.right_enc.steps

        d_left_steps = (left_steps - self.last_left_steps) * self.left_dir
        d_right_steps = (right_steps - self.last_right_steps) * self.right_dir

        self.last_left_steps = left_steps
        self.last_right_steps = right_steps

        # Convert steps -> meters
        wheel_circ = math.pi * self.wheel_diameter
        meters_per_tick = wheel_circ / self.ticks_per_rev

        dl = d_left_steps * meters_per_tick
        dr = d_right_steps * meters_per_tick

        # --- "PAC-MAN MODE" DEBUG VERSION ---
        
        # 1. Calculate the Raw turning vs. moving amounts
        # We use absolute values to compare magnitude
        turn_magnitude = abs(dr - dl)
        move_magnitude = abs(dr + dl)

        # 2. The Check: Is the robot mostly turning?
        # We relax the check slightly (0.8) to catch sloppy turns
        if turn_magnitude > (move_magnitude * 0.8): 
            # LOGGING: Verify this is happening!
            # self.get_logger().info(f"SPINNING! dl:{dl:.4f} dr:{dr:.4f} -> FORCING ds=0")
            
            ds = 0.0      # <--- THIS STOPS THE SLIDING
            
            # Use your tuned rotation formula (dr - dl) or (dl - dr)
            # You said dr - dl was better for you previously:
            dtheta = (dr - dl) / self.wheel_separation 
            
        else:
            # Normal driving
            ds = (dr + dl) / 2.0
            dtheta = (dr - dl) / self.wheel_separation

        # Calculate raw contributions
        # diff = dr - dl      # Rotation component (raw)
        # summ = dr + dl      # Forward component (raw)
        
        # # Check: Is the robot mostly turning?
        # # If the 'turning amount' (diff) is larger than the 'forward amount' (summ),
        # # we assume it's INTENDED to be a spin-in-place.
        # if abs(diff) > abs(summ):
        #     ds = 0.0        # FORCE forward movement to zero
        #     dtheta = diff / self.wheel_separation  # Pure rotation
        # else:
        #     ds = summ / 2.0 # Normal driving
        #     dtheta = diff / self.wheel_separation
            
        # --- END FIX ---

        # Diff-drive integration
        # if (dl > 0 and dr < 0) or (dl < 0 and dr > 0):
        #     # We are turning in place! 
        #     # Force linear distance (ds) to ZERO to stop the "Walking/Drifting" on map.
        #     ds = 0.0
        #     # Trust the rotation completely
        #     dtheta = (dr - dl) / self.wheel_separation # (Use dl-dr or dr-dl based on your fix)
            
        # else:
        #     # Normal driving (Straight or Arcs)
        #     ds = (dr + dl) / 2.0
        #     dtheta = (dr - dl) / self.wheel_separation
        # ds = (dr + dl) / 2.0
        # dtheta = (dr - dl) / self.wheel_separation

        # Update pose using midpoint yaw
        yaw_mid = self.yaw + dtheta / 2.0
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw = (self.yaw + dtheta + math.pi) % (2.0 * math.pi) - math.pi

        vx = ds / dt
        vth = dtheta / dt

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(vth)

        self.odom_pub.publish(odom)

        # Publish TF
        if self.publish_tf and self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            q = yaw_to_quat(self.yaw)
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
