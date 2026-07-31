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

        # Initialize hardware
        # Note: gpiozero 'distance' property returns meters by default
        self.sensor = DistanceSensor(echo=21, trigger=20, max_distance=2.0)

        # <--- CHANGED: Publisher uses Range type
        self.publisher_ = self.create_publisher(Range, 'ultrasonic_distance', 10)
        
        # Reduced timer to 0.1s (10Hz) for faster obstacle reaction
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Ultrasonic node started. Publishing Range messages.')

    def timer_callback(self):
        # Read distance (gpiozero returns meters)
        try:
            current_distance = self.sensor.distance
        except Exception as e:
            self.get_logger().warn(f"Sensor error: {e}")
            return

        # Create the Range message required by Nav2
        msg = Range()
        
        # 1. TIME & FRAME (Critical for Costmaps)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_link' # <--- MUST MATCH YOUR URDF LINK NAME
        
        # 2. SENSOR PROPERTIES
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 degrees in radians (standard for HC-SR04)
        msg.min_range = 0.02      # 2 cm
        msg.max_range = 2.0       # 2 meters
        
        # 3. THE DATA (Must be in Meters)
        if current_distance >= 1.95: # Close to max distance means no echo / out of range
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()