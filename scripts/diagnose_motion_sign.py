#!/usr/bin/env python3
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


def yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalized(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("motion_sign_probe")
        # Exercise the same safety path used by Nav2.
        self.command = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.create_subscription(Odometry, "/icp/odom", self.on_odom, 20)
        self.create_subscription(Float32MultiArray, "/hardware/status", self.on_status, 20)
        self.create_subscription(Float32, "/hardware/battery_raw", self.on_battery, 20)
        self.odom = None
        self.samples = []
        self.voltages = []

    def on_odom(self, message: Odometry) -> None:
        self.odom = message

    def on_status(self, message: Float32MultiArray) -> None:
        if len(message.data) >= 6:
            self.samples.append(list(message.data[:6]))

    def on_battery(self, message: Float32) -> None:
        self.voltages.append(float(message.data))


def spin_for(node: Probe, seconds: float, command: Twist | None = None) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if command is not None:
            node.command.publish(command)
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> None:
    rclpy.init()
    node = Probe()
    stop = Twist()
    try:
        spin_for(node, 1.5, stop)
        if node.odom is None:
            raise RuntimeError("no /icp/odom")
        before = yaw(node.odom)
        turn = Twist()
        turn.angular.z = 0.30
        node.samples.clear()
        spin_for(node, 1.0, turn)
        spin_for(node, 0.8, stop)
        if node.odom is None:
            raise RuntimeError("lost /icp/odom")
        after = yaw(node.odom)
        active = [sample for sample in node.samples if abs(sample[0]) + abs(sample[3]) > 0.1]
        peak_command = max(active, key=lambda sample: abs(sample[0]) + abs(sample[3])) if active else None
        peak_feedback = max(active, key=lambda sample: abs(sample[1]) + abs(sample[4])) if active else None
        print(json.dumps({
            "command_angular_z": turn.angular.z,
            "yaw_before": before,
            "yaw_after": after,
            "yaw_delta": normalized(after - before),
            "peak_command_status": peak_command,
            "peak_feedback_status": peak_feedback,
            "minimum_voltage": min(node.voltages) if node.voltages else None,
            "samples": len(node.samples),
        }, ensure_ascii=False))
    finally:
        for _ in range(5):
            node.command.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
