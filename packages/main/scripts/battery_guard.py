#!/usr/bin/env python3
"""Global low-battery interlock for Nav2 and UI goals."""

import json
import time

import rclpy
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger


ACTION_NAMES = (
    "/navigate_to_pose",
    "/navigate_through_poses",
    "/follow_path",
    "/spin",
    "/backup",
    "/drive_on_heading",
    "/assisted_teleop",
    "/wait",
)


class BatteryGuard(Node):
    def __init__(self):
        super().__init__("battery_guard")
        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.low = False
        self.voltage = None
        self.cancel_generation = 0
        self.last_spoken = float("-inf")
        self.speech_interval = 60.0
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.voice_pub = self.create_publisher(String, "/voice/speak", 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.status_pub = self.create_publisher(String, "/safety/status", safety_qos)
        self.stop_pub = self.create_publisher(Bool, "/safety/stop", safety_qos)
        self.create_subscription(
            Bool, "/hardware/low_battery", self.low_battery_callback, safety_qos
        )
        self.create_subscription(Float32, "/hardware/battery_raw", self.voltage_callback, 10)
        self.goal_cancel = self.create_client(Trigger, "/goal/cancel")
        self.action_cancels = {
            name: self.create_client(CancelGoal, f"{name}/_action/cancel_goal")
            for name in ACTION_NAMES
        }
        self.create_timer(0.1, self.enforce_stop)
        self.create_timer(1.0, self.periodic_update)
        self.stop_pub.publish(Bool(data=False))
        self.publish_status("ready")

    def voltage_callback(self, message: Float32):
        self.voltage = float(message.data)

    def publish_status(self, state: str):
        payload = {"state": state, "voltage": self.voltage}
        text = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(String(data=text))
        self.get_logger().warning(text)

    def low_battery_callback(self, message: Bool):
        if message.data and not self.low:
            self.low = True
            self.cancel_generation += 1
            self.stop_pub.publish(Bool(data=True))
            self.publish_status("low_battery_stop")
            self.announce_battery_replacement()
            self.cancel_all_tasks()
        elif not message.data and self.low:
            self.low = False
            self.stop_pub.publish(Bool(data=False))
            self.publish_status("battery_recovered")

    def enforce_stop(self):
        if self.low:
            self.cmd_pub.publish(Twist())

    def periodic_update(self):
        self.publish_diagnostics()
        # Repeating while latched also rejects goals submitted after the fault.
        if self.low:
            self.cancel_all_tasks()
            if time.monotonic() - self.last_spoken >= self.speech_interval:
                self.announce_battery_replacement()

    def announce_battery_replacement(self):
        self.last_spoken = time.monotonic()
        self.voice_pub.publish(
            String(data="Низкое напряжение аккумулятора. Замените аккумулятор.")
        )

    def publish_diagnostics(self):
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.ERROR if self.low else DiagnosticStatus.OK
        status.name = "RoboMarvel/Battery"
        status.hardware_id = "STM motor controller"
        status.message = (
            "Низкое напряжение. Замените аккумулятор. Текущая задача отменена."
            if self.low
            else "Напряжение аккумулятора в рабочем диапазоне"
        )
        status.values = [
            KeyValue(key="voltage", value=str(self.voltage)),
            KeyValue(key="action", value="replace_battery" if self.low else "none"),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_pub.publish(message)

    def cancel_all_tasks(self):
        if self.goal_cancel.service_is_ready():
            self.goal_cancel.call_async(Trigger.Request())
        request = CancelGoal.Request()  # all-zero GoalInfo means cancel all goals
        for client in self.action_cancels.values():
            if client.service_is_ready():
                client.call_async(request)


def main():
    rclpy.init()
    node = BatteryGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
