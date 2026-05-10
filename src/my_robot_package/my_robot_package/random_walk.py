#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range
import random
import time
import math

class RandomWalk(Node):
    def __init__(self):
        super().__init__('random_walk')
        
        # Publisher to drive the motors
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribe to ALL your sensors
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.sub_ir_l = self.create_subscription(Range, 'ir/left', self.range_cb, 10)
        self.sub_ir_r = self.create_subscription(Range, 'ir/right', self.range_cb, 10)
        self.sub_ultra = self.create_subscription(Range, 'ultrasonic_distance', self.range_cb, 10)

        # State tracking
        self.last_obstacle_time = 0.0
        self.state = 'FORWARD'
        self.state_end_time = 0.0
        self.turn_speed = 0.0

        # Run the brain at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Random Walk Brain Initialized. Let's map!")

    def range_cb(self, msg):
        # Trigger if IR or Ultrasonic sees something between 5cm and 40cm
        if 0.05 < msg.range < 0.40:
            self.last_obstacle_time = time.time()

    def scan_cb(self, msg):
        # Trigger if LiDAR sees something in the front 60-degree cone
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        for i, r in enumerate(msg.ranges):
            angle = angle_min + i * angle_inc
            
            # Normalize angle to -pi to pi
            while angle > math.pi: angle -= 2 * math.pi
            while angle < -math.pi: angle += 2 * math.pi
            
            # If it's looking roughly forward (+/- 30 degrees)
            if abs(angle) < 0.52: 
                # Ignore the robot's own 13cm chassis, trigger if object is < 45cm
                if 0.15 < r < 0.45: 
                    self.last_obstacle_time = time.time()
                    break # Stop checking if we found one hit

    def control_loop(self):
        msg = Twist()
        now = time.time()
        
        # If an obstacle was seen in the last 0.2 seconds
        obstacle_detected = (now - self.last_obstacle_time) < 0.2

        if self.state == 'FORWARD':
            if obstacle_detected:
                self.state = 'TURN'
                # Pick a random turn duration (between 1.5 and 3.5 seconds)
                self.state_end_time = now + random.uniform(1.5, 3.5)
                # Pick a random direction (left or right)
                self.turn_speed = random.choice([-0.4, 0.4])
                self.get_logger().warn('Obstacle detected! Evading...')
            else:
                msg.linear.x = 0.12  # Safe driving speed
                msg.angular.z = 0.0

        elif self.state == 'TURN':
            if now < self.state_end_time:
                msg.linear.x = 0.0
                msg.angular.z = self.turn_speed
            else:
                self.state = 'FORWARD'
                self.get_logger().info('Path clear, resuming exploration.')

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Exploration stopped by user.')
    finally:
        # Send a stop command before dying
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()