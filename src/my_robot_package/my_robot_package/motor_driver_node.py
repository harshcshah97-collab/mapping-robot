#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import DigitalOutputDevice


class MotorDriverNode(Node):
    """
    Simple differential drive using your tested pins:

      IN1 = GPIO23
      IN2 = GPIO22
      IN3 = GPIO17
      IN4 = GPIO27

    Listens to /cmd_vel and turns motors on/off.
    No PWM yet – just basic forward/back/turn.
    """

    def __init__(self):
        super().__init__('motor_driver_node')

        # Motor pins
        self.left_in1 = DigitalOutputDevice(17)   # IN1
        self.left_in2 = DigitalOutputDevice(27)   # IN2
        self.right_in1 = DigitalOutputDevice(23)  # IN3
        self.right_in2 = DigitalOutputDevice(22)  # IN4

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("Motor driver node started, listening to /cmd_vel")

    # --- Low-level helpers ---

    def stop(self):
        self.left_in1.off()
        self.left_in2.off()
        self.right_in1.off()
        self.right_in2.off()

    def drive_forward(self):
        # Both motors forward
        self.left_in1.on()
        self.left_in2.off()
        self.right_in1.on()
        self.right_in2.off()

    def drive_backward(self):
        # Both motors backward
        self.left_in1.off()
        self.left_in2.on()
        self.right_in1.off()
        self.right_in2.on()

    def turn_left_in_place(self):
        # Left motor backward, right motor forward
        self.left_in1.on()
        self.left_in2.off()
        self.right_in1.off()
        self.right_in2.on()

    def turn_right_in_place(self):
        # Left motor forward, right motor backward
        self.left_in1.off()
        self.left_in2.on()
        self.right_in1.on()
        self.right_in2.off()

    # --- /cmd_vel callback ---

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        # Small deadband to avoid jitter
        if abs(linear) < 0.05 and abs(angular) < 0.05:
            self.stop()
            return

        # If strong angular and small linear: turn in place
        if abs(angular) > 0.3 and abs(linear) < 0.02:
            if angular > 0:
                self.turn_left_in_place()
                self.get_logger().debug("Turning left in place")
            else:
                self.turn_right_in_place()
                self.get_logger().debug("Turning right in place")
            return

        # Otherwise, go forward/back
        if linear > 0:
            self.drive_forward()
            self.get_logger().debug("Driving forward")
        elif linear < 0:
            self.drive_backward()
            self.get_logger().debug("Driving backward")

    def destroy_node(self):
        self.get_logger().info("Cleaning up motor GPIO...")
        self.stop()
        self.left_in1.close()
        self.left_in2.close()
        self.right_in1.close()
        self.right_in2.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
