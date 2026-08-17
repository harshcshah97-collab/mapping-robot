#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
import math
import time
from gpiozero import Button


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.declare_parameter('sensor_timeout_sec', 1.0)
        self.declare_parameter('minimum_lidar_range_m', 0.05)
        self.declare_parameter('stop_on_return_to_start', True)
        self.declare_parameter('left_bumper_gpio', 9)
        self.declare_parameter('center_bumper_gpio', 11)
        self.declare_parameter('right_bumper_gpio', 10)
        self.sensor_timeout_sec = float(
            self.get_parameter('sensor_timeout_sec').value
        )
        self.minimum_lidar_range_m = float(
            self.get_parameter('minimum_lidar_range_m').value
        )
        self.stop_on_return_to_start = bool(
            self.get_parameter('stop_on_return_to_start').value
        )

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.create_subscription(Range, 'ir/left', self.ir_left_cb, 10)
        self.create_subscription(Range, 'ir/right', self.ir_right_cb, 10)
        self.create_subscription(Range, 'ultrasonic_distance', self.ultra_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.left_bumper = Button(
            int(self.get_parameter('left_bumper_gpio').value),
            pull_up=True,
            bounce_time=0.1,
        )
        self.center_bumper = Button(
            int(self.get_parameter('center_bumper_gpio').value),
            pull_up=True,
            bounce_time=0.1,
        )
        self.right_bumper = Button(
            int(self.get_parameter('right_bumper_gpio').value),
            pull_up=True,
            bounce_time=0.1,
        )

        self.front_dist = 1.0
        self.right_dist = 1.0
        self.left_dist = 1.0
        self.front_right_dist = 1.0

        # NEW: Store the actual raw distance from the IR sensors
        self.ir_left_dist = 3.0
        self.ir_right_dist = 3.0

        self.ir_left_blocked = False
        self.ir_right_blocked = False
        self.ultra_blocked = False
        self.wall_found = False
        self.last_scan_time = None

        self.escape_phase = 0
        self.escape_counter = 0
        self.escape_turn_speed = 0.0

        self.start_x = None
        self.start_y = None
        self.start_time = None
        self.left_starting_zone = False
        self.map_complete = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Smart Mapper Online. Full Sensor Fusion ACTIVE.")

    def odom_cb(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        if self.start_x is None and self.start_y is None:
            self.start_x = current_x
            self.start_y = current_y
            self.start_time = self.get_clock().now()
            return
        dist_from_start = math.hypot(current_x - self.start_x, current_y - self.start_y)
        if not self.left_starting_zone and dist_from_start > 2.5:
            self.left_starting_zone = True
            self.get_logger().info("Robot has left the starting zone. Return tracker ARMED.")

        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if (
            self.stop_on_return_to_start
            and self.left_starting_zone
            and not self.map_complete
            and dist_from_start < 0.5
        ):
            if elapsed_time > 300.0:  # 5 minutes timer
                self.map_complete = True
                self.get_logger().warning(
                    "Returned near the start after 5 minutes; stopping mapping motion."
                )

    # --- UPDATED CALLBACKS: Store the raw range value ---
    def ir_left_cb(self, msg):
        self.ir_left_dist = msg.range
        self.ir_left_blocked = 0.05 < msg.range < 0.20
        self.get_logger().debug(f"IR Left: {msg.range:.2f} m, Blocked: {self.ir_left_blocked}")

    def ir_right_cb(self, msg):
        self.ir_right_dist = msg.range
        self.ir_right_blocked = 0.05 < msg.range < 0.20
        self.get_logger().debug(f"IR Right: {msg.range:.2f} m, Blocked: {self.ir_right_blocked}")

    def ultra_cb(self, msg):
        self.ultra_blocked = 0.05 < msg.range < 0.25
        self.get_logger().debug(f"Ultrasonic: {msg.range:.2f} m, Blocked: {self.ultra_blocked}")

    def sector_min(self, message, center_degrees, half_width_degrees):
        center = math.radians(center_degrees)
        half_width = math.radians(half_width_degrees)
        minimum_valid = max(message.range_min, self.minimum_lidar_range_m)
        values = []
        for index, distance in enumerate(message.ranges):
            if not math.isfinite(distance):
                continue
            if not minimum_valid <= distance <= message.range_max:
                continue
            angle = message.angle_min + index * message.angle_increment
            difference = math.atan2(
                math.sin(angle - center), math.cos(angle - center)
            )
            if abs(difference) <= half_width:
                values.append(distance)
        return min(values) if values else 3.0

    def scan_cb(self, msg):
        self.front_dist = self.sector_min(msg, 0.0, 10.0)
        self.right_dist = self.sector_min(msg, -90.0, 10.0)
        self.front_right_dist = self.sector_min(msg, -45.0, 7.5)
        self.left_dist = self.sector_min(msg, 90.0, 10.0)
        self.last_scan_time = time.monotonic()
        self.get_logger().debug(
            f"LIDAR: front={self.front_dist:.2f}m, right={self.right_dist:.2f}m, "
            f"left={self.left_dist:.2f}m, front_right={self.front_right_dist:.2f}m"
        )

    def control_loop(self):
        msg = Twist()
        if (
            self.last_scan_time is None
            or time.monotonic() - self.last_scan_time > self.sensor_timeout_sec
        ):
            self.publisher_.publish(msg)
            return

        # --- 0. LOOP CLOSURE OVERRIDE ---
        if self.map_complete:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            return

        # --- 1. ODOMETRY-SAFE BUMPER ESCAPE ---
        if self.escape_phase == 1:
            msg.linear.x = -0.12
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.escape_counter -= 1
            if self.escape_counter <= 0:
                self.escape_phase = 2
                self.escape_counter = 25
            return
        elif self.escape_phase == 2:
            msg.linear.x = 0.0
            msg.angular.z = self.escape_turn_speed
            self.publisher_.publish(msg)
            self.escape_counter -= 1
            if self.escape_counter <= 0:
                self.escape_phase = 0
            return

        if self.left_bumper.is_pressed or self.center_bumper.is_pressed:
            self.publisher_.publish(msg)
            self.escape_phase = 1
            self.escape_counter = 10
            self.escape_turn_speed = -0.4
            return
        elif self.right_bumper.is_pressed:
            self.publisher_.publish(msg)
            self.escape_phase = 1
            self.escape_counter = 10
            self.escape_turn_speed = 0.4
            return

        # --- 2. NON-CONTACT SENSOR OVERRIDE ---
        if self.ultra_blocked:
            msg.linear.x = 0.0
            # SENSOR FUSION: Find the absolute closest threat on each side
            true_left_clearance = min(self.left_dist, self.ir_left_dist)
            true_right_clearance = min(self.right_dist, self.ir_right_dist)

            if true_left_clearance > true_right_clearance:
                msg.angular.z = 0.4
            else:
                msg.angular.z = -0.4
            self.publisher_.publish(msg)
            return

        if self.ir_left_blocked:
            msg.linear.x = 0.0
            msg.angular.z = -0.5
            self.publisher_.publish(msg)
            return
        elif self.ir_right_blocked:
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.publisher_.publish(msg)
            return

        # --- 3. STATE 0: SEEK AND LOCK ---
        if not self.wall_found:
            if self.front_dist < 0.4 or self.right_dist < 0.4 or self.front_right_dist < 0.4:
                self.wall_found = True
            else:
                msg.linear.x = 0.15
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
                return

        # --- 4. PERIMETER SWEEP (Wall Following) ---
        if self.front_dist < 0.35:
            msg.linear.x = 0.0

            # SENSOR FUSION: Find the absolute closest threat on each side
            true_left_clearance = min(self.left_dist, self.ir_left_dist)
            true_right_clearance = min(self.right_dist, self.ir_right_dist)

            if true_left_clearance > true_right_clearance:
                msg.angular.z = 0.4   # <--- SMOOTHED
            else:
                msg.angular.z = -0.4  # <--- SMOOTHED

        elif self.right_dist < 0.20:
            msg.linear.x = 0.0
            msg.angular.z = 0.4
        elif self.right_dist > 0.30 and self.right_dist < 1.0:
            msg.linear.x = 0.0
            msg.angular.z = -0.4
        elif self.right_dist >= 0.20 and self.right_dist <= 0.30:
            msg.linear.x = 0.15
            msg.angular.z = 0.0       # Coast straight
        elif self.right_dist >= 1.0:
            msg.linear.x = 0.0
            msg.angular.z = -0.4

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.publisher_.publish(Twist())
        for bumper in (self.left_bumper, self.center_bumper, self.right_bumper):
            bumper.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
