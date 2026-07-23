import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


class Message:
    def __init__(self, data=None):
        self.data = data


class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Twist:
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class Publisher:
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeGpio:
    IN1, IN2, IN3, IN4 = 1, 2, 3, 4
    TRIG, ECHO, IRF_R, IRF_L = 5, 6, 7, 8

    def __init__(self):
        self.pins = {}
        self.inputs = {self.IRF_R: 1, self.IRF_L: 1, 24: 1, 23: 1}
        self.left_pwm = 0
        self.right_pwm = 0

    def digital_write(self, pin, value):
        self.pins[pin] = value

    def digital_read(self, pin):
        return self.inputs.get(pin, 0)

    def ena_pwm(self, value):
        self.left_pwm = value

    def enb_pwm(self, value):
        self.right_pwm = value


def install_stubs():
    rospy = types.ModuleType("rospy")
    rospy.logerr = lambda *args, **kwargs: None
    rospy.logwarn = lambda *args, **kwargs: None
    rospy.logwarn_throttle = lambda *args, **kwargs: None
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.Publisher = Publisher
    rospy.Subscriber = lambda *args, **kwargs: None
    rospy.Timer = lambda *args, **kwargs: None
    rospy.Duration = lambda value: value
    rospy.on_shutdown = lambda callback: None
    rospy.init_node = lambda *args, **kwargs: None
    rospy.get_param = lambda _name, default: default
    rospy.spin = lambda: None
    sys.modules["rospy"] = rospy

    geometry = types.ModuleType("geometry_msgs.msg")
    geometry.Twist = Twist
    geometry.Vector3 = Vector3
    geometry.Quaternion = Quaternion
    sys.modules["geometry_msgs"] = types.ModuleType("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = geometry

    std = types.ModuleType("std_msgs.msg")
    std.Bool = Message
    std.Float32MultiArray = Message
    std.Int32 = Message
    std.String = Message
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std


def load_bridge():
    install_stubs()
    path = Path(__file__).parents[1] / "pi" / "gfsx_unity_safe_bridge.py"
    spec = importlib.util.spec_from_file_location("enhanced_bridge", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class EnhancedBridgeTests(unittest.TestCase):
    def setUp(self):
        self.bridge = load_bridge()
        self.bridge.gpio = FakeGpio()
        self.bridge.has_gpio = True
        self.bridge.pwm_publisher = Publisher()
        self.bridge.drive_armed_publisher = Publisher()
        self.bridge.sensor_publisher = Publisher()
        self.bridge.gripper_ir_publisher = Publisher()
        self.bridge.rear_ir_publisher = Publisher()

    def set_local_sensors(
        self,
        *,
        updated=10.0,
        sonar=1.0,
        left=0,
        right=0,
        rear=0,
    ):
        self.bridge.last_local_sensor_update = updated
        self.bridge.local_ultrasonic_metres = sonar
        self.bridge.local_left_ir = left
        self.bridge.local_right_ir = right
        self.bridge.local_rear_ir = rear

    def send_velocity(self, *, linear, angular=0.0, now=10.0):
        self.bridge.drive_enabled = True
        self.bridge.last_drive_enable = now
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        with mock.patch.object(self.bridge.time, "monotonic", return_value=now):
            self.bridge.velocity_callback(message)

    def test_drive_ack_is_authoritative_and_false_before_true(self):
        with mock.patch.object(self.bridge.time, "monotonic", return_value=10.0):
            self.bridge.drive_enable_callback(Message(False))
        self.assertFalse(self.bridge.drive_enabled)
        self.assertFalse(self.bridge.drive_armed_publisher.messages[-1].data)
        with mock.patch.object(self.bridge.time, "monotonic", return_value=10.1):
            self.bridge.drive_enable_callback(Message(True))
        self.assertTrue(self.bridge.drive_enabled)
        self.assertTrue(self.bridge.drive_armed_publisher.messages[-1].data)

    def test_drive_timeout_publishes_negative_ack(self):
        self.bridge.drive_enabled = True
        self.bridge.last_drive_enable = 1.0
        self.bridge.last_cmd_vel = 1.0
        with mock.patch.object(self.bridge.time, "monotonic", return_value=2.0):
            self.bridge.safety_timer(None)
        self.assertFalse(self.bridge.drive_enabled)
        self.assertFalse(self.bridge.drive_armed_publisher.messages[-1].data)

    def test_nonfinite_angular_velocity_disarms_and_hard_stops(self):
        self.bridge.drive_enabled = True
        self.bridge.drive_rearm_ready = False
        self.bridge.previous_pwm_left = -35.0
        self.bridge.previous_pwm_right = 35.0
        message = Twist()
        message.angular.z = float("inf")

        self.bridge.velocity_callback(message)

        self.assertFalse(self.bridge.drive_enabled)
        self.assertFalse(self.bridge.drive_rearm_ready)
        self.assertEqual(
            (self.bridge.gpio.left_pwm, self.bridge.gpio.right_pwm),
            (0, 0),
        )
        self.assertFalse(self.bridge.drive_armed_publisher.messages[-1].data)
        self.assertEqual(
            (self.bridge.pwm_publisher.messages[-1].x,
             self.bridge.pwm_publisher.messages[-1].y),
            (0.0, 0.0),
        )

    def test_missing_sonar_echo_becomes_blocked_zero(self):
        self.bridge.last_ultrasonic_valid = 0.0
        with mock.patch.object(self.bridge, "read_ultrasonic_cm", return_value=None):
            with mock.patch.object(self.bridge.time, "monotonic", return_value=10.0):
                self.bridge.sensor_timer(None)
        message = self.bridge.sensor_publisher.messages[-1]
        self.assertEqual(message.x, 0.0)
        self.assertEqual((message.y, message.z, message.w), (0.0, 0.0, 0.0))
        self.assertEqual(self.bridge.last_local_sensor_update, 10.0)
        self.assertEqual(self.bridge.local_ultrasonic_metres, 0.0)

    def test_stale_local_sensors_block_translation_but_allow_pure_pivot(self):
        self.set_local_sensors(updated=9.0)

        self.send_velocity(linear=0.2, now=10.0)
        self.assertEqual((self.bridge.gpio.left_pwm, self.bridge.gpio.right_pwm), (0, 0))

        self.send_velocity(linear=-0.2, now=10.0)
        self.assertEqual((self.bridge.gpio.left_pwm, self.bridge.gpio.right_pwm), (0, 0))

        self.send_velocity(linear=0.0, angular=1.0, now=10.0)
        self.assertEqual(
            (self.bridge.applied_pwm_left, self.bridge.applied_pwm_right),
            (35.0, -35.0),
        )

    def test_deployable_bridge_hard_caps_each_track_at_35_pwm(self):
        for _ in range(8):
            self.bridge.set_motors_pwm(-100.0, 100.0)

        self.assertEqual(self.bridge.MAX_MOTOR_PWM, 35.0)
        self.assertEqual(
            (self.bridge.gpio.left_pwm, self.bridge.gpio.right_pwm),
            (35, 35),
        )
        self.assertEqual(
            (self.bridge.applied_pwm_left, self.bridge.applied_pwm_right),
            (-35.0, 35.0),
        )
        self.assertEqual(
            (self.bridge.previous_pwm_left, self.bridge.previous_pwm_right),
            (-35.0, 35.0),
        )
        self.assertTrue(all(
            abs(message.x) <= self.bridge.MAX_MOTOR_PWM
            and abs(message.y) <= self.bridge.MAX_MOTOR_PWM
            for message in self.bridge.pwm_publisher.messages
        ))

    def test_forward_translation_is_blocked_by_sonar_and_either_front_ir(self):
        hazards = (
            {"sonar": 0.12},
            {"sonar": 1.0, "left": 1},
            {"sonar": 1.0, "right": 1},
        )
        for hazard in hazards:
            with self.subTest(hazard=hazard):
                self.bridge.hard_stop_motors()
                self.set_local_sensors(**hazard)
                self.send_velocity(linear=0.2)
                self.assertEqual(
                    (self.bridge.gpio.left_pwm, self.bridge.gpio.right_pwm),
                    (0, 0),
                )

    def test_forward_clearance_above_threshold_is_allowed(self):
        self.set_local_sensors(sonar=0.1201)

        self.send_velocity(linear=0.2)

        self.assertEqual(
            (self.bridge.applied_pwm_left, self.bridge.applied_pwm_right),
            (35.0, 35.0),
        )

    def test_reverse_uses_rear_ir_not_front_obstacle_sensors(self):
        self.set_local_sensors(sonar=0.0, left=1, right=1, rear=0)
        self.send_velocity(linear=-0.2)
        self.assertEqual(
            (self.bridge.applied_pwm_left, self.bridge.applied_pwm_right),
            (-35.0, -35.0),
        )

        self.bridge.hard_stop_motors()
        self.set_local_sensors(sonar=1.0, left=0, right=0, rear=1)
        self.send_velocity(linear=-0.2)
        self.assertEqual((self.bridge.gpio.left_pwm, self.bridge.gpio.right_pwm), (0, 0))


if __name__ == "__main__":
    unittest.main()
