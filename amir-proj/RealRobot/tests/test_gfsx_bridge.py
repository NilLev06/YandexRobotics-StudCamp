import importlib.util
import sys
import types
import unittest
from unittest import mock
from pathlib import Path


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

    def __init__(self):
        self.pins = {}
        self.left_pwm = 0
        self.right_pwm = 0

    def digital_write(self, pin, value):
        self.pins[pin] = value

    def ena_pwm(self, value):
        self.left_pwm = value

    def enb_pwm(self, value):
        self.right_pwm = value


class FakeServo:
    def __init__(self):
        self.commands = []

    def set(self, channel, angle):
        self.commands.append((channel, angle))


def install_ros_stubs():
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
    install_ros_stubs()
    path = Path(__file__).parents[1] / "pi" / "gfsx_unity_safe_bridge.py"
    spec = importlib.util.spec_from_file_location("bridge_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class BridgeLogicTests(unittest.TestCase):
    def setUp(self):
        self.bridge = load_bridge()
        self.bridge.gpio = FakeGpio()
        self.bridge.has_gpio = True
        self.bridge.servo = FakeServo()
        self.bridge.has_servo = True
        self.bridge.pwm_publisher = Publisher()
        self.bridge.status_publisher = Publisher()
        self.bridge.servo_armed_publisher = Publisher()
        self.bridge.drive_armed_publisher = Publisher()

    def test_hardware_preflight_refuses_degraded_bridge(self):
        self.bridge.has_gpio = False
        self.bridge.gpio = None
        self.bridge.has_servo = False
        self.bridge.servo = None
        self.bridge.ultrasonic = None

        self.assertEqual(
            self.bridge.hardware_preflight_errors(),
            [
                "xr_gpio/GPIO",
                "xr_servo/servo controller",
                "xr_ultrasonic/ultrasonic sensor",
            ],
        )

        self.bridge.has_gpio = True
        self.bridge.gpio = FakeGpio()
        self.bridge.has_servo = True
        self.bridge.servo = FakeServo()
        self.bridge.ultrasonic = object()
        self.assertEqual(self.bridge.hardware_preflight_errors(), [])

    def motor_directions(self, left, right):
        self.bridge.previous_pwm_left = 0.0
        self.bridge.previous_pwm_right = 0.0
        self.bridge.set_motors_pwm(left, right)
        gpio = self.bridge.gpio
        return (
            gpio.pins[gpio.IN1] - gpio.pins[gpio.IN2],
            gpio.pins[gpio.IN3] - gpio.pins[gpio.IN4],
        )

    def test_corrected_drive_truth_table(self):
        self.assertEqual(self.motor_directions(50, 50), (1, 1))
        self.assertEqual(self.motor_directions(-50, -50), (-1, -1))
        self.assertEqual(self.motor_directions(50, -50), (1, -1))
        self.assertEqual(self.motor_directions(-50, 50), (-1, 1))

    def test_dead_zone_boundary_is_stopped_and_breakaway_pwm_is_reported(self):
        self.bridge.previous_pwm_left = 0.0
        self.bridge.previous_pwm_right = 0.0
        self.bridge.set_motors_pwm(10.0, -10.0)
        self.assertEqual(self.bridge.gpio.left_pwm, 0)
        self.assertEqual(self.bridge.gpio.right_pwm, 0)
        self.assertEqual(
            (self.bridge.pwm_publisher.messages[-1].x,
             self.bridge.pwm_publisher.messages[-1].y),
            (0.0, 0.0),
        )

        self.bridge.set_motors_pwm(10.5, -10.5)
        self.assertEqual(self.bridge.gpio.left_pwm, 35)
        self.assertEqual(self.bridge.gpio.right_pwm, 35)
        self.assertEqual(
            (self.bridge.pwm_publisher.messages[-1].x,
             self.bridge.pwm_publisher.messages[-1].y),
            (35.0, -35.0),
        )

    def test_final_per_track_pwm_never_exceeds_hardware_cap(self):
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

    def test_hard_stop_clears_smoothed_turn_and_applied_pwm(self):
        self.bridge.previous_angular = 0.21
        self.bridge.applied_pwm_left = 35.0
        self.bridge.applied_pwm_right = -35.0

        self.bridge.hard_stop_motors()

        self.assertEqual(self.bridge.previous_angular, 0.0)
        self.assertEqual(self.bridge.applied_pwm_left, 0.0)
        self.assertEqual(self.bridge.applied_pwm_right, 0.0)
        self.assertEqual(
            (self.bridge.pwm_publisher.messages[-1].x,
             self.bridge.pwm_publisher.messages[-1].y),
            (0.0, 0.0),
        )

    def test_nonfinite_velocity_disarms_and_hard_stops(self):
        self.bridge.drive_enabled = True
        self.bridge.drive_rearm_ready = False
        self.bridge.previous_pwm_left = 35.0
        self.bridge.previous_pwm_right = -35.0
        message = Twist()
        message.linear.x = float("nan")

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

    def test_absolute_servo_targets_are_clamped_atomically(self):
        now = self.bridge.time.monotonic()
        self.bridge.servo_enabled = True
        self.bridge.last_servo_enable = now
        self.bridge.desired_servo_angles = [90.0] * 6
        self.bridge.servo_targets_callback(
            Message([100.0, 200.0, -10.0, 181.0, 180.0, 0.0]))
        self.assertEqual(
            self.bridge.desired_servo_angles,
            [100.0, 180.0, 0.0, 50.0, 160.0, 0.0],
        )
        previous = list(self.bridge.desired_servo_angles)
        self.bridge.servo_targets_callback(Message([1.0, 2.0]))
        self.assertEqual(self.bridge.desired_servo_angles, previous)

    def test_drive_and_servo_watchdogs_are_separate(self):
        self.assertEqual(self.bridge.DRIVE_ENABLE_TIMEOUT, 0.45)
        self.assertEqual(self.bridge.COMMAND_TIMEOUT, 0.45)
        self.assertEqual(self.bridge.SERVO_ENABLE_TIMEOUT, 1.5)
        self.assertEqual(self.bridge.SERVO_REARM_DISARM_DWELL, 1.0)

    def test_completed_disarm_invalidates_the_previous_servo_target(self):
        self.bridge.servo_enabled = True
        self.bridge.servo_target_received = True

        self.bridge.servo_enable_callback(Message(False))

        self.assertFalse(self.bridge.servo_enabled)
        self.assertFalse(self.bridge.servo_target_received)

    def test_disarmed_target_is_stored_and_applied_only_after_rearm(self):
        self.bridge.current_servo_angles = [90.0, 90.0, 90.0, 0.0, 90.0, 90.0]
        self.bridge.desired_servo_angles = [90.0, 90.0, 90.0, 0.0, 90.0, 90.0]

        self.bridge.servo_targets_callback(
            Message([92.0, 88.0, 90.0, 20.0, 91.0, 89.0]))

        self.assertTrue(self.bridge.servo_target_received)
        self.assertEqual(
            self.bridge.desired_servo_angles,
            [92.0, 88.0, 90.0, 20.0, 91.0, 89.0],
        )
        self.bridge.servo_timer(None)
        self.assertEqual(self.bridge.servo.commands, [])

        # A True without a preceding False remains fail-closed.
        self.bridge.servo_enable_callback(Message(True))
        self.assertFalse(self.bridge.servo_enabled)
        self.bridge.servo_timer(None)
        self.assertEqual(self.bridge.servo.commands, [])

        # A continuous False baseline must last for the full safety dwell.
        # The following True then arms the already-stored complete target,
        # eliminating cross-topic ordering without accepting a lone packet.
        with mock.patch.object(
            self.bridge.time,
            "monotonic",
            side_effect=[100.0, 101.1, 101.2, 101.3],
        ):
            self.bridge.servo_enable_callback(Message(False))
            self.bridge.servo_enable_callback(Message(False))
            self.bridge.servo_enable_callback(Message(True))
            self.bridge.servo_timer(None)
        self.assertTrue(self.bridge.servo_enabled)

        self.assertIn((1, 92), self.bridge.servo.commands)
        self.assertIn((2, 88), self.bridge.servo.commands)
        self.assertIn((7, 91), self.bridge.servo.commands)
        self.assertIn((8, 89), self.bridge.servo.commands)
        self.assertTrue(self.bridge.servo_armed_publisher.messages[-1].data)

    def test_claw_and_pan_are_clamped_to_physical_ranges(self):
        now = self.bridge.time.monotonic()
        self.bridge.servo_enabled = True
        self.bridge.last_servo_enable = now
        self.bridge.servo_targets_callback(Message([90, 90, 90, 180, 180, 90]))
        self.assertEqual(self.bridge.desired_servo_angles[3], 50.0)
        self.assertEqual(self.bridge.desired_servo_angles[4], 160.0)
        self.bridge.servo_targets_callback(Message([90, 90, 90, 0, 0, 90]))
        self.assertEqual(self.bridge.desired_servo_angles[3], 0.0)
        self.assertEqual(self.bridge.desired_servo_angles[4], 15.0)

    def test_claw_uses_reduced_slew_rate(self):
        now = self.bridge.time.monotonic()
        self.bridge.servo_enabled = True
        self.bridge.last_servo_enable = now
        self.bridge.servo_target_received = True
        self.bridge.current_servo_angles = [90.0, 90.0, 90.0, 0.0, 90.0, 90.0]
        self.bridge.desired_servo_angles = [90.0, 90.0, 90.0, 50.0, 90.0, 90.0]

        self.bridge.servo_timer(None)

        self.assertEqual(self.bridge.current_servo_angles[3], 1.0)
        self.assertIn((4, 1), self.bridge.servo.commands)

    def test_sensor_head_uses_this_robots_physical_ports_7_and_8(self):
        self.assertEqual(
            self.bridge.SERVO_CHANNELS,
            (1, 2, 3, 4, 7, 8),
        )
        now = self.bridge.time.monotonic()
        self.bridge.servo_enabled = True
        self.bridge.last_servo_enable = now
        self.bridge.servo_target_received = True
        self.bridge.current_servo_angles = [90.0, 90.0, 90.0, 0.0, 90.0, 90.0]
        self.bridge.desired_servo_angles = [90.0, 90.0, 90.0, 0.0, 92.0, 88.0]

        self.bridge.servo_timer(None)

        self.assertIn((7, 92), self.bridge.servo.commands)
        self.assertIn((8, 88), self.bridge.servo.commands)

    def test_idle_servo_state_heartbeat_republishes_all_six_values(self):
        self.bridge.current_servo_angles = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        self.bridge.servo_state_publisher = Publisher()

        self.bridge.servo_state_heartbeat(None)

        self.assertEqual(len(self.bridge.servo_state_publisher.messages), 1)
        self.assertEqual(
            self.bridge.servo_state_publisher.messages[0].data,
            [10.0, 20.0, 30.0, 40.0, 50.0, 60.0],
        )

    def test_timeout_requires_false_before_rearm(self):
        self.bridge.servo_enabled = True
        self.bridge.servo_rearm_ready = False
        self.bridge.last_servo_enable = 0.0
        self.bridge.safety_timer(None)
        self.assertFalse(self.bridge.servo_enabled)
        self.assertFalse(self.bridge.servo_armed_publisher.messages[-1].data)
        self.assertIn(
            "timed out",
            self.bridge.status_publisher.messages[-1].data,
        )
        self.bridge.servo_enable_callback(Message(True))
        self.assertFalse(self.bridge.servo_enabled)
        self.assertFalse(self.bridge.servo_armed_publisher.messages[-1].data)
        with mock.patch.object(
            self.bridge.time,
            "monotonic",
            side_effect=[200.0, 201.1, 201.2],
        ):
            self.bridge.servo_enable_callback(Message(False))
            self.bridge.servo_enable_callback(Message(False))
            self.bridge.servo_enable_callback(Message(True))
        self.assertTrue(self.bridge.servo_enabled)
        self.assertTrue(self.bridge.servo_armed_publisher.messages[-1].data)

    def test_alternating_false_true_publishers_cannot_rearm(self):
        self.bridge.servo_enabled = False
        self.bridge.servo_rearm_ready = False
        self.bridge.servo_disarm_since = None

        with mock.patch.object(
            self.bridge.time,
            "monotonic",
            side_effect=[300.0, 300.1, 300.2, 300.3, 300.4, 300.5],
        ):
            for requested in (False, True, False, True, False, True):
                self.bridge.servo_enable_callback(Message(requested))

        self.assertFalse(self.bridge.servo_enabled)
        self.assertFalse(self.bridge.servo_rearm_ready)
        self.assertFalse(self.bridge.servo_armed_publisher.messages[-1].data)


if __name__ == "__main__":
    unittest.main()
