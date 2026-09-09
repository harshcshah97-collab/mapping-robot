#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from gpiozero import DigitalInputDevice


class IRPublisher(Node):
    def __init__(self):
        super().__init__('ir_sensor_node')

        # --- 1. NEW: Declare and Read Parameters ---
        self.declare_parameter('left_frame_id', 'ir_left_link')
        self.declare_parameter('right_frame_id', 'ir_right_link')
        self.declare_parameter('left_gpio', 16)
        self.declare_parameter('right_gpio', 26)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.frame_left = self.get_parameter('left_frame_id').value
        self.frame_right = self.get_parameter('right_frame_id').value

        # --- PHYSICAL CONFIGURATION ---
        self.pin_left = int(self.get_parameter('left_gpio').value)
        self.pin_right = int(self.get_parameter('right_gpio').value)

        self.sensor_left = DigitalInputDevice(self.pin_left)
        self.sensor_right = DigitalInputDevice(self.pin_right)

        self.pub_left = self.create_publisher(Range, '/ir/left', 10)
        self.pub_right = self.create_publisher(Range, '/ir/right', 10)

        publish_rate_hz = max(
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )
        self.timer = self.create_timer(
            1.0 / publish_rate_hz, self.publish_sensor_data
        )
        self.get_logger().info(f"IR Driver Started. Left Frame: {self.frame_left}, Right Frame: {self.frame_right}")

    def create_range_msg(self, frame_id, sensor):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.61
        msg.min_range = 0.02
        msg.max_range = 0.30

        if sensor.value == 0:
            msg.range = 0.05
        else:
            msg.range = float('inf')

        return msg

    def publish_sensor_data(self):
        # --- 2. NEW: Use the parameter variables here ---
        left_msg = self.create_range_msg(self.frame_left, self.sensor_left)
        right_msg = self.create_range_msg(self.frame_right, self.sensor_right)

        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)

    def destroy_node(self):
        self.sensor_left.close()
        self.sensor_right.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IRPublisher()
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
