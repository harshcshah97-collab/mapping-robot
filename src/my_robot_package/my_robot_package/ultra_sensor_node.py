#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range  # <--- CHANGED: Import Range for Nav2 compatibility
from gpiozero import DistanceSensor


class UltraSensorNode(Node):
    """
    Ultrasonic distance node using gpiozero DistanceSensor.

    Publishes sensor_msgs/Range for Nav2 compatibility.
    """

    def __init__(self):
        super().__init__('ultra_sensor_node')
        self.declare_parameter('frame_id', 'ultrasonic_link')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('echo_gpio', 21)
        self.declare_parameter('trigger_gpio', 20)
        self.declare_parameter('max_distance_m', 2.0)
        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_rate_hz = max(
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )

        # Initialize hardware
        # Note: gpiozero 'distance' property returns meters by default
        max_distance_m = float(self.get_parameter('max_distance_m').value)
        self.max_distance_m = max_distance_m
        self.sensor = DistanceSensor(
            echo=int(self.get_parameter('echo_gpio').value),
            trigger=int(self.get_parameter('trigger_gpio').value),
            max_distance=max_distance_m,
        )

        # <--- CHANGED: Publisher uses Range type
        self.publisher_ = self.create_publisher(Range, 'ultrasonic_distance', 10)

        # Reduced timer to 0.1s (10Hz) for faster obstacle reaction
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)

        self.get_logger().info('Ultrasonic node started. Publishing Range messages.')

    def timer_callback(self):
        # Read distance (gpiozero returns meters)
        try:
            current_distance = self.sensor.distance
        except Exception as e:
            self.get_logger().warning(f"Sensor error: {e}")
            return

        # Create the Range message required by Nav2
        msg = Range()

        # 1. TIME & FRAME (Critical for Costmaps)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # 2. SENSOR PROPERTIES
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 degrees in radians (standard for HC-SR04)
        msg.min_range = 0.02      # 2 cm
        msg.max_range = self.max_distance_m

        # 3. THE DATA (Must be in Meters)
        if current_distance >= msg.max_range * 0.975:
            msg.range = float('inf')
        else:
            msg.range = float(current_distance)

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.sensor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltraSensorNode()
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
