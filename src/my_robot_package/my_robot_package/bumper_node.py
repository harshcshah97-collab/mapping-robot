#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from gpiozero import Button
import math

class BumperNode(Node):
    def __init__(self):
        super().__init__('bumper_node')
        
        # Publish the obstacle points
        self.cloud_pub = self.create_publisher(PointCloud2, '/bumper_cloud', 10)
        
        # Initialize physical bumper switches
        self.left_bumper = Button(9, pull_up=True, bounce_time=0.1)
        self.center_bumper = Button(11, pull_up=True, bounce_time=0.1)
        self.right_bumper = Button(10, pull_up=True, bounce_time=0.1)

        # Timer to constantly check switches and publish
        self.timer = self.create_timer(0.05, self.check_bumpers) # Runs at 20Hz
        self.get_logger().info("Bumper Translator Online. Pain Receptors Active.")

    def check_bumpers(self):
        points = []
        
        # If a bumper is pressed, create an obstacle point at that physical location.
        # These coordinates are relative to the center of your robot base.
        # (Assuming your robot radius is roughly 0.15m)
        
        if self.center_bumper.is_pressed:
            points.append([0.16, 0.0, 0.0])   # Directly in front
            
        if self.left_bumper.is_pressed:
            points.append([0.15, 0.1, 0.0])   # Front-Left corner
            
        if self.right_bumper.is_pressed:
            points.append([0.15, -0.1, 0.0])  # Front-Right corner

        # If any points were created, publish them!
        if points:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'base_footprint' 
            
            cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
            self.cloud_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BumperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()