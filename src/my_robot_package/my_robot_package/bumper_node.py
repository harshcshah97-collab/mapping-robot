#!/usr/bin/env python3
import rclpy
from rclpy.exceptions import RCLError
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from gpiozero import Button


class BumperNode(Node):
    def __init__(self):
        super().__init__('bumper_node')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('left_gpio', 9)
        self.declare_parameter('center_gpio', 11)
        self.declare_parameter('right_gpio', 10)
        self.frame_id = str(self.get_parameter('frame_id').value)

        # Publish the obstacle points
        self.cloud_pub = self.create_publisher(PointCloud2, '/bumper_cloud', 10)
        self.stop_pub = self.create_publisher(Bool, '/safety/stop', 10)

        # Initialize physical bumper switches
        self.left_bumper = Button(
            int(self.get_parameter('left_gpio').value),
            pull_up=True,
            bounce_time=0.1,
        )
        self.center_bumper = Button(
            int(self.get_parameter('center_gpio').value),
            pull_up=True,
            bounce_time=0.1,
        )
        self.right_bumper = Button(
            int(self.get_parameter('right_gpio').value),
            pull_up=True,
            bounce_time=0.1,
        )

        # Timer to constantly check switches and publish
        self.timer = self.create_timer(0.05, self.check_bumpers)  # Runs at 20Hz
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

        # The motor driver consumes this directly; Nav2 costmap updates alone
        # are not fast enough to serve as a contact emergency stop.
        self.stop_pub.publish(Bool(data=bool(points)))

        # If any points were created, publish them!
        if points:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.frame_id

            cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
            self.cloud_pub.publish(cloud_msg)

    def destroy_node(self):
        try:
            if self.context.ok():
                try:
                    self.stop_pub.publish(Bool(data=True))
                except RCLError:
                    pass
            for bumper in (self.left_bumper, self.center_bumper, self.right_bumper):
                bumper.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BumperNode()
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
