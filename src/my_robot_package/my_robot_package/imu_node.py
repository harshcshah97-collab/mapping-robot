#!/usr/bin/env python3
import math
import smbus
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# MPU6050 Registers & constants (from your imu_test_node2.py)
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

ACCEL_SENSITIVITY = 16384.0  # LSB/g
GYRO_SENSITIVITY  = 131.0    # LSB/(°/s)

# Use your calibration offsets
GYRO_OFFSET_X = -5.74 * GYRO_SENSITIVITY
GYRO_OFFSET_Y = -0.79 * GYRO_SENSITIVITY
GYRO_OFFSET_Z = -0.57 * GYRO_SENSITIVITY


def read_raw_data(bus, addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) + low
    if value > 32768:
        value -= 65536
    return value


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.bus = smbus.SMBus(1)  # I2C bus 1
        # Wake MPU6050
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        self.get_logger().info("IMU node started (MPU6050 @ 0x68 on I2C bus 1)")

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Accelerometer in g
        accel_x = read_raw_data(self.bus, ACCEL_XOUT_H)       / ACCEL_SENSITIVITY
        accel_y = read_raw_data(self.bus, ACCEL_XOUT_H + 2)   / ACCEL_SENSITIVITY
        accel_z = read_raw_data(self.bus, ACCEL_XOUT_H + 4)   / ACCEL_SENSITIVITY

        # Convert to m/s^2
        g = 9.80665
        msg.linear_acceleration.x = accel_x * g
        msg.linear_acceleration.y = accel_y * g
        msg.linear_acceleration.z = accel_z * g

        # Gyro with offsets, convert deg/s -> rad/s
        raw_gyro_x = read_raw_data(self.bus, GYRO_XOUT_H)       - GYRO_OFFSET_X
        raw_gyro_y = read_raw_data(self.bus, GYRO_XOUT_H + 2)   - GYRO_OFFSET_Y
        raw_gyro_z = read_raw_data(self.bus, GYRO_XOUT_H + 4)   - GYRO_OFFSET_Z

        gyro_x = (raw_gyro_x / GYRO_SENSITIVITY) * math.pi / 180.0
        gyro_y = (raw_gyro_y / GYRO_SENSITIVITY) * math.pi / 180.0
        gyro_z = (raw_gyro_z / GYRO_SENSITIVITY) * math.pi / 180.0

        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z

        # Orientation not estimated here
        msg.orientation_covariance[0] = -1.0

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down IMU node")
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
