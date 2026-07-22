#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


rclpy.init()
node = Node("publish_battery_alert")
publisher = node.create_publisher(String, "/voice/speak", 10)
deadline = time.monotonic() + 2.0
while time.monotonic() < deadline and publisher.get_subscription_count() == 0:
    rclpy.spin_once(node, timeout_sec=0.1)
publisher.publish(
    String(data="Низкое напряжение аккумулятора. Замените аккумулятор.")
)
rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
