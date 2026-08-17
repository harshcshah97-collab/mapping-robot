#!/usr/bin/env python3
import math
import smbus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# MPU6050 Registers & constants (from your imu_test_node2.py)
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

ACCEL_SENSITIVITY = 16384.0  # LSB/g
GYRO_SENSITIVITY = 131.0    # LSB/(°/s)


def read_raw_data(bus, device_address, register):
    high = bus.read_byte_data(device_address, register)
    low = bus.read_byte_data(device_address, register + 1)
    value = (high << 8) + low
    if value >= 32768:
        value -= 65536
    return value


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', MPU6050_ADDR)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('gyro_offset_x_deg_s', -5.74)
        self.declare_parameter('gyro_offset_y_deg_s', -0.79)
        self.declare_parameter('gyro_offset_z_deg_s', -0.57)
        self.i2c_bus = int(self.get_parameter('i2c_bus').value)
        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.gyro_offsets = (
            float(self.get_parameter('gyro_offset_x_deg_s').value),
            float(self.get_parameter('gyro_offset_y_deg_s').value),
            float(self.get_parameter('gyro_offset_z_deg_s').value),
        )
        publish_rate_hz = max(
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )

        self.bus = smbus.SMBus(self.i2c_bus)
        # Wake MPU6050
        self.bus.write_byte_data(self.i2c_address, PWR_MGMT_1, 0)

        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)

        self.get_logger().info(
            "IMU node started (MPU6050 @ 0x%02x on I2C bus %d)"
            % (self.i2c_address, self.i2c_bus)
        )

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        try:
            accel_x = read_raw_data(
                self.bus, self.i2c_address, ACCEL_XOUT_H
            ) / ACCEL_SENSITIVITY
            accel_y = read_raw_data(
                self.bus, self.i2c_address, ACCEL_XOUT_H + 2
            ) / ACCEL_SENSITIVITY
            accel_z = read_raw_data(
                self.bus, self.i2c_address, ACCEL_XOUT_H + 4
            ) / ACCEL_SENSITIVITY
            raw_gyro_x = read_raw_data(
                self.bus, self.i2c_address, GYRO_XOUT_H
            ) - self.gyro_offsets[0] * GYRO_SENSITIVITY
            raw_gyro_y = read_raw_data(
                self.bus, self.i2c_address, GYRO_XOUT_H + 2
            ) - self.gyro_offsets[1] * GYRO_SENSITIVITY
            raw_gyro_z = read_raw_data(
                self.bus, self.i2c_address, GYRO_XOUT_H + 4
            ) - self.gyro_offsets[2] * GYRO_SENSITIVITY
        except OSError as error:
            self.get_logger().error(
                "MPU6050 read failed: %s" % error,
                throttle_duration_sec=2.0,
            )
            return

        # Convert to m/s^2
        g = 9.80665
        msg.linear_acceleration.x = accel_x * g
        msg.linear_acceleration.y = accel_y * g
        msg.linear_acceleration.z = accel_z * g

        # Gyro with offsets, convert deg/s -> rad/s
        gyro_x = (raw_gyro_x / GYRO_SENSITIVITY) * math.pi / 180.0
        gyro_y = (raw_gyro_y / GYRO_SENSITIVITY) * math.pi / 180.0
        gyro_z = (raw_gyro_z / GYRO_SENSITIVITY) * math.pi / 180.0

        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z

        # Orientation not estimated here
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02
        msg.linear_acceleration_covariance[0] = 0.10
        msg.linear_acceleration_covariance[4] = 0.10
        msg.linear_acceleration_covariance[8] = 0.10

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
