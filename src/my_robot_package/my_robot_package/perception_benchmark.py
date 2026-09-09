#!/usr/bin/env python3
"""Measure OAK-D point-cloud throughput and Raspberry Pi thermal headroom."""

import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


def cpu_temperature_c():
    """Read the usual Linux thermal-zone value, or return ``None``."""
    thermal_path = Path("/sys/class/thermal/thermal_zone0/temp")
    try:
        return float(thermal_path.read_text().strip()) / 1000.0
    except (OSError, ValueError):
        return None


class PerceptionBenchmark(Node):
    """Count point clouds and LiDAR scans without altering robot state."""

    def __init__(self):
        super().__init__("perception_benchmark")
        self.pointcloud_count = 0
        self.scan_count = 0
        self.point_count = 0
        self.create_subscription(
            PointCloud2,
            "/oakd/depth/points",
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )

    def pointcloud_callback(self, message):
        self.pointcloud_count += 1
        self.point_count += int(message.width) * int(message.height)

    def scan_callback(self, _message):
        self.scan_count += 1


def main(arguments=None):
    parser = argparse.ArgumentParser(
        description="Benchmark OAK-D depth publication on the robot."
    )
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--minimum-cloud-hz", type=float, default=5.0)
    parser.add_argument("--maximum-temperature-c", type=float, default=75.0)
    args, ros_arguments = parser.parse_known_args(arguments)
    if args.duration <= 0.0:
        parser.error("--duration must be positive")

    rclpy.init(args=ros_arguments)
    node = PerceptionBenchmark()
    started = time.monotonic()
    max_temperature = cpu_temperature_c()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
            temperature = cpu_temperature_c()
            if temperature is not None:
                max_temperature = (
                    temperature
                    if max_temperature is None
                    else max(max_temperature, temperature)
                )
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = max(time.monotonic() - started, 1e-6)
        result = {
            "duration_sec": round(elapsed, 2),
            "oakd_cloud_hz": round(node.pointcloud_count / elapsed, 2),
            "average_points_per_cloud": (
                round(node.point_count / node.pointcloud_count)
                if node.pointcloud_count
                else 0
            ),
            "lidar_scan_hz": round(node.scan_count / elapsed, 2),
            "maximum_cpu_temperature_c": (
                round(max_temperature, 1) if max_temperature is not None else None
            ),
            "one_minute_load_average": round(os.getloadavg()[0], 2),
        }
        print(json.dumps(result, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    cloud_rate_ok = result["oakd_cloud_hz"] >= args.minimum_cloud_hz
    temperature_ok = (
        max_temperature is None
        or max_temperature <= args.maximum_temperature_c
    )
    return 0 if cloud_rate_ok and temperature_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
