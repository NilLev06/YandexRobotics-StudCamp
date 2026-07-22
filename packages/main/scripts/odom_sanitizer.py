#!/usr/bin/env python3
"""Validate RTAB-Map ICP odometry and own the odom -> base_link transform.

RTAB-Map may briefly publish an all-zero quaternion while it resets. Google
Cartographer treats that quaternion as a fatal error. This node rejects invalid
samples, normalizes valid quaternions, and keeps the public odometry continuous
across ICP resets by maintaining a 2-D offset.
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

Pose2D = tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    c = math.cos(a[2])
    s = math.sin(a[2])
    return (
        a[0] + c * b[0] - s * b[1],
        a[1] + s * b[0] + c * b[1],
        normalize_angle(a[2] + b[2]),
    )


def inverse(pose: Pose2D) -> Pose2D:
    c = math.cos(pose[2])
    s = math.sin(pose[2])
    return (
        -c * pose[0] - s * pose[1],
        s * pose[0] - c * pose[1],
        normalize_angle(-pose[2]),
    )


class OdomSanitizer(Node):
    def __init__(self) -> None:
        super().__init__("odom_sanitizer")
        self.declare_parameter("input_topic", "/icp/odom_raw")
        self.declare_parameter("output_topic", "/icp/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("jump_translation_m", 1.0)
        self.declare_parameter("jump_rotation_rad", 2.4)
        self.declare_parameter("invalid_recovery_window_sec", 3.0)
        self.declare_parameter("tf_republish_hz", 20.0)
        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("tf_stamp_delay_sec", 0.15)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.jump_translation = float(self.get_parameter("jump_translation_m").value)
        self.jump_rotation = float(self.get_parameter("jump_rotation_rad").value)
        self.recovery_window = float(
            self.get_parameter("invalid_recovery_window_sec").value
        )
        self.stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        self.tf_stamp_delay = float(self.get_parameter("tf_stamp_delay_sec").value)

        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self._on_odom, 20
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.offset: Pose2D = (0.0, 0.0, 0.0)
        self.last_raw: Optional[Pose2D] = None
        self.last_stable: Optional[Pose2D] = None
        self.last_output: Optional[Odometry] = None
        self.last_valid_time = self.get_clock().now()
        self.invalid_seen_at = None
        self.invalid_count = 0
        self.reset_count = 0

        frequency = max(1.0, float(self.get_parameter("tf_republish_hz").value))
        self.create_timer(1.0 / frequency, self._republish_tf)
        self.get_logger().info(
            f"Sanitizing {self.input_topic} -> {self.output_topic}; "
            f"publishing TF {self.odom_frame} -> {self.base_frame}"
        )

    def _valid_pose(self, msg: Odometry) -> tuple[bool, Pose2D | None]:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            return False, None
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1.0e-6:
            return False, None
        qx, qy, qz, qw = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        return True, (p.x, p.y, quaternion_to_yaw(qx, qy, qz, qw))

    def _on_odom(self, msg: Odometry) -> None:
        valid, raw = self._valid_pose(msg)
        now = self.get_clock().now()
        if not valid or raw is None:
            self.invalid_count += 1
            self.invalid_seen_at = now
            if self.invalid_count == 1 or self.invalid_count % 50 == 0:
                self.get_logger().warning(
                    "Rejected invalid ICP odometry quaternion/pose "
                    f"(count={self.invalid_count})"
                )
            return

        should_realign = False
        if self.last_raw is not None and self.last_stable is not None:
            raw_delta = compose(inverse(self.last_raw), raw)
            translation_jump = math.hypot(raw_delta[0], raw_delta[1])
            rotation_jump = abs(normalize_angle(raw_delta[2]))
            recent_invalid = False
            if self.invalid_seen_at is not None:
                age = (now - self.invalid_seen_at).nanoseconds * 1.0e-9
                recent_invalid = age <= self.recovery_window
            should_realign = (
                recent_invalid
                or translation_jump > self.jump_translation
                or rotation_jump > self.jump_rotation
            )

        if should_realign and self.last_stable is not None:
            self.offset = compose(self.last_stable, inverse(raw))
            self.reset_count += 1
            self.get_logger().warning(
                "ICP reset/jump detected; preserving continuous odom frame "
                f"(realignments={self.reset_count})"
            )
            self.invalid_seen_at = None

        stable = compose(self.offset, raw)
        output = Odometry()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        output.pose.pose.position.x = stable[0]
        output.pose.pose.position.y = stable[1]
        output.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(stable[2])
        output.pose.pose.orientation.x = qx
        output.pose.pose.orientation.y = qy
        output.pose.pose.orientation.z = qz
        output.pose.pose.orientation.w = qw
        output.pose.covariance = list(msg.pose.covariance)
        output.twist = msg.twist

        self.publisher.publish(output)
        self._broadcast(stable, msg.header.stamp)
        self.last_raw = raw
        self.last_stable = stable
        self.last_output = output
        self.last_valid_time = now

    def _broadcast(self, pose: Pose2D, stamp) -> None:
        transform = TransformStamped()
        transform.header.stamp = (Time.from_msg(stamp) - Duration(seconds=self.tf_stamp_delay)).to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose[0]
        transform.transform.translation.y = pose[1]
        qx, qy, qz, qw = yaw_to_quaternion(pose[2])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _republish_tf(self) -> None:
        if self.last_stable is None:
            return
        age = (self.get_clock().now() - self.last_valid_time).nanoseconds * 1.0e-9
        if age > self.stale_timeout:
            return
        self._broadcast(self.last_stable, self.last_output.header.stamp)


def main() -> None:
    rclpy.init()
    node = OdomSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
