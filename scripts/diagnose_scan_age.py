#!/usr/bin/env python3
import json
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanAgeProbe(Node):
    def __init__(self) -> None:
        super().__init__("scan_age_probe")
        self.ages = []
        self.arrivals = []
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, message: LaserScan) -> None:
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.ages.append((self.get_clock().now().nanoseconds - stamp_ns) / 1e9)
        self.arrivals.append(time.monotonic())


def percentile(values, ratio):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int((len(ordered) - 1) * ratio))]


def main() -> None:
    rclpy.init()
    node = ScanAgeProbe()
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    gaps = [b - a for a, b in zip(node.arrivals, node.arrivals[1:])]
    print(json.dumps({
        "samples": len(node.ages),
        "age_min_s": min(node.ages) if node.ages else None,
        "age_median_s": statistics.median(node.ages) if node.ages else None,
        "age_p95_s": percentile(node.ages, 0.95) if node.ages else None,
        "age_max_s": max(node.ages) if node.ages else None,
        "arrival_gap_p95_s": percentile(gaps, 0.95) if gaps else None,
        "arrival_gap_max_s": max(gaps) if gaps else None,
    }))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
