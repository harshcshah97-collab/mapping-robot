#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from my_robot_package.motor_logic import (
    feedforward_pwm,
    limit_wheel_targets,
    pi_pwm,
    wheel_speeds,
    wheel_targets,
)


class MotorDriverNode(Node):
    """
    Drive a differential base using independent, signed wheel PWM.

      left IN1/IN2 = GPIO23/GPIO22
      right IN1/IN2 = GPIO17/GPIO27

    Robot launch files set the command input to the arbitrated ``/cmd_vel_out``.
    The default remains ``/cmd_vel`` so the existing standalone dashboard
    teleop process continues to work without a dashboard change. Open-loop PWM
    is always available; encoder-based PI feedback is calibration-gated.
    """

    def __init__(self):
        super().__init__('motor_driver_node')

        self.declare_parameter('command_timeout_sec', 0.6)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('wheel_separation_m', 0.240)
        self.declare_parameter('max_wheel_speed_mps', 0.22)
        self.declare_parameter('minimum_pwm', 0.28)
        self.declare_parameter('closed_loop_enabled', False)
        self.declare_parameter('wheel_calibration_confirmed', False)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.2)
        self.declare_parameter('integral_limit', 0.4)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('left_in1_gpio', 23)
        self.declare_parameter('left_in2_gpio', 22)
        self.declare_parameter('right_in1_gpio', 17)
        self.declare_parameter('right_in2_gpio', 27)
        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value
        )
        self.control_rate_hz = max(
            1.0, float(self.get_parameter('control_rate_hz').value)
        )
        self.wheel_separation_m = float(
            self.get_parameter('wheel_separation_m').value
        )
        self.max_wheel_speed_mps = float(
            self.get_parameter('max_wheel_speed_mps').value
        )
        self.minimum_pwm = float(self.get_parameter('minimum_pwm').value)
        requested_closed_loop = bool(
            self.get_parameter('closed_loop_enabled').value
        )
        calibration_confirmed = bool(
            self.get_parameter('wheel_calibration_confirmed').value
        )
        self.closed_loop_enabled = requested_closed_loop and calibration_confirmed
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.integral_limit = float(self.get_parameter('integral_limit').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        if self.wheel_separation_m <= 0.0 or self.max_wheel_speed_mps <= 0.0:
            raise ValueError('Wheel separation and maximum speed must be positive')
        if not 0.0 <= self.minimum_pwm <= 1.0:
            raise ValueError('minimum_pwm must be between 0 and 1')

        self.last_command_time = time.monotonic()
        self.last_control_time = self.last_command_time
        self.motors_are_stopped = True
        self.safety_stop_active = False
        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        self.measured_left_mps = 0.0
        self.measured_right_mps = 0.0
        self.left_integral = 0.0
        self.right_integral = 0.0

        # Motor pins
        self.left_motor = Motor(
            forward=int(self.get_parameter('left_in1_gpio').value),
            backward=int(self.get_parameter('left_in2_gpio').value),
            pwm=True,
        )
        self.right_motor = Motor(
            forward=int(self.get_parameter('right_in1_gpio').value),
            backward=int(self.get_parameter('right_in2_gpio').value),
            pwm=True,
        )

        self.subscription = self.create_subscription(
            Twist,
            self.command_topic,
            self.cmd_vel_callback,
            10
        )
        self.safety_subscription = self.create_subscription(
            Bool,
            '/safety/stop',
            self.safety_stop_callback,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz, self.control_callback
        )

        if requested_closed_loop and not calibration_confirmed:
            self.get_logger().warning(
                'Closed-loop control was requested but wheel calibration is not '
                'confirmed; using open-loop PWM.'
            )
        self.get_logger().info(
            'Motor PWM ready on %s (%s loop).'
            % (
                self.command_topic,
                'closed' if self.closed_loop_enabled else 'open',
            )
        )

    # --- Low-level helpers ---

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        self.left_integral = 0.0
        self.right_integral = 0.0
        self.motors_are_stopped = True

    @staticmethod
    def _set_motor(motor, signed_pwm):
        if signed_pwm > 0.0:
            motor.forward(min(signed_pwm, 1.0))
        elif signed_pwm < 0.0:
            motor.backward(min(abs(signed_pwm), 1.0))
        else:
            motor.stop()

    # --- Arbitrated Twist and encoder feedback callbacks ---

    def cmd_vel_callback(self, msg: Twist):
        self.last_command_time = time.monotonic()
        if self.safety_stop_active:
            self.stop()
            return
        target_left, target_right = wheel_targets(
            float(msg.linear.x),
            float(msg.angular.z),
            self.wheel_separation_m,
        )
        self.target_left_mps, self.target_right_mps = limit_wheel_targets(
            target_left, target_right, self.max_wheel_speed_mps
        )

    def odom_callback(self, message):
        self.measured_left_mps, self.measured_right_mps = wheel_speeds(
            float(message.twist.twist.linear.x),
            float(message.twist.twist.angular.z),
            self.wheel_separation_m,
        )

    def control_callback(self):
        now = time.monotonic()
        dt = max(now - self.last_control_time, 1e-6)
        self.last_control_time = now
        command_age = now - self.last_command_time
        if self.safety_stop_active or command_age > self.command_timeout_sec:
            if not self.motors_are_stopped and command_age > self.command_timeout_sec:
                self.get_logger().warning(
                    'No fresh %s for %.2fs; watchdog stopped the base.'
                    % (self.command_topic, command_age)
                )
            self.stop()
            return

        if self.closed_loop_enabled:
            left_pwm, self.left_integral = pi_pwm(
                self.target_left_mps, self.measured_left_mps,
                self.left_integral, dt, self.max_wheel_speed_mps,
                self.minimum_pwm, self.kp, self.ki, self.integral_limit,
            )
            right_pwm, self.right_integral = pi_pwm(
                self.target_right_mps, self.measured_right_mps,
                self.right_integral, dt, self.max_wheel_speed_mps,
                self.minimum_pwm, self.kp, self.ki, self.integral_limit,
            )
        else:
            left_pwm = feedforward_pwm(
                self.target_left_mps, self.max_wheel_speed_mps, self.minimum_pwm
            )
            right_pwm = feedforward_pwm(
                self.target_right_mps, self.max_wheel_speed_mps, self.minimum_pwm
            )

        self._set_motor(self.left_motor, left_pwm)
        self._set_motor(self.right_motor, right_pwm)
        self.motors_are_stopped = left_pwm == 0.0 and right_pwm == 0.0

    def safety_stop_callback(self, message):
        self.safety_stop_active = bool(message.data)
        if self.safety_stop_active:
            self.stop()

    def destroy_node(self):
        self.get_logger().info("Cleaning up motor GPIO...")
        self.stop()
        self.left_motor.close()
        self.right_motor.close()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
