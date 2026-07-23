#!/usr/bin/env python3
"""Safe ROS bridge for the GFS-X real robot.

This node never moves a servo during startup. Drive commands require an
enable heartbeat, and the motors are stopped if either heartbeat or cmd_vel
messages stop. Translation is also blocked against the Pi's own fresh
sonar/front/rear IR snapshot. Servo commands are absolute physical controller
degrees, and startup fails if required hardware drivers do not initialize.
"""

import atexit
import json
import math
import os
import sys
import threading
import time
import traceback

import rospy
from geometry_msgs.msg import Quaternion, Twist, Vector3
from std_msgs.msg import Bool, Float32MultiArray, Int32, String


sys.path.append("/root/XiaoRGeek")

STATE_PATH = "/root/gfsx_servo_state.json"
CMD_VEL_TOPIC = "/gfsx/cmd_vel"
DRIVE_ENABLE_TOPIC = "/gfsx/drive_enable"
SERVO_ENABLE_TOPIC = "/gfsx/servo_enable"
SERVO_TOPIC = "/gfsx/servo_targets_degrees"
SERVO_STATE_TOPIC = "/gfsx/servo_state_degrees"
SERVO_ARMED_TOPIC = "/gfsx/servo_armed"
DRIVE_ARMED_TOPIC = "/gfsx/drive_armed"
# Actual custom wiring on this robot (not the stock manual layout):
# IO1 = physical right IR (xr_gpio.IRF_L / BCM1)
# IO2 = physical left IR  (xr_gpio.IRF_R / BCM25)
# IO3 = claw IR          (BCM24)
# IO4 = physical rear IR (BCM23)
GRIPPER_IR_PIN = 24
REAR_IR_PIN = 23

DRIVE_ENABLE_TIMEOUT = 0.45
DEFAULT_SERVO_ENABLE_TIMEOUT = 1.5
SERVO_ENABLE_TIMEOUT = DEFAULT_SERVO_ENABLE_TIMEOUT
DEFAULT_SERVO_REARM_DISARM_DWELL = 1.0
SERVO_REARM_DISARM_DWELL = DEFAULT_SERVO_REARM_DISARM_DWELL
COMMAND_TIMEOUT = 0.45
MAX_LINEAR = 0.25
TURN_K = 0.25
PWM_CONVERSION_FACTOR = 200.0
MIN_MOTOR_PWM = 35.0
MAX_MOTOR_PWM = 35.0
MOTOR_DEAD_ZONE = 10.0
MAX_PWM_STEP = 15.0
LOCAL_SENSOR_TIMEOUT = 0.40
MIN_FORWARD_CLEARANCE_METRES = 0.12

# The ROS payload has six logical controls: S1-S4 arm/claw followed by
# sensor pan and camera pitch. On this specific robot the two sensor-head
# leads are physically plugged into controller sockets 7 and 8. Keep the
# six-value ROS ordering while translating the final two values here.
SERVO_CHANNELS = (1, 2, 3, 4, 7, 8)
SERVO_LIMITS = (
    (0.0, 180.0),
    (0.0, 180.0),
    (0.0, 180.0),
    (0.0, 50.0),   # S4 claw: 0 open, 50 closed on this robot.
    (15.0, 160.0), # S5 sensor-head pan.
    (0.0, 180.0),
)
MAX_SERVO_STEPS = (
    2.0,
    2.0,
    2.0,
    1.0,  # S4: limit the first physical calibration to about 20 deg/s.
    2.0,
    2.0,
)

gpio = None
servo = None
has_gpio = False
has_servo = False
ultrasonic = None
sensor_publisher = None
gripper_ir_publisher = None
rear_ir_publisher = None
ultrasonic_history = [500.0, 500.0, 500.0]
last_ultrasonic_valid = 0.0
left_ir_history = [0, 0, 0]
right_ir_history = [0, 0, 0]
gripper_ir_history = [0, 0, 0]
rear_ir_history = [0, 0, 0]

# This copy of the filtered physical sensor state lives in the motor process,
# so translation remains fail-closed even when telemetry from the Pi to Unity
# is delayed.  The zero timestamp intentionally blocks translation until the
# first complete local sensor sample; an in-place pivot does not translate the
# chassis and remains available to turn away from an obstacle.
last_local_sensor_update = 0.0
local_ultrasonic_metres = 0.0
local_left_ir = 1
local_right_ir = 1
local_rear_ir = 1

try:
    import xr_gpio as gpio

    has_gpio = True
    gpio.GPIO.setup(GRIPPER_IR_PIN, gpio.GPIO.IN, pull_up_down=gpio.GPIO.PUD_UP)
    gpio.GPIO.setup(REAR_IR_PIN, gpio.GPIO.IN, pull_up_down=gpio.GPIO.PUD_UP)
except Exception:
    rospy.logerr("Could not load xr_gpio:\n%s", traceback.format_exc())

try:
    # xr_servo reads these values on every set(). Override them only inside
    # this dedicated ROS bridge process; other robot programs keep their own
    # XiaoR configuration.
    import xr_config as servo_config

    servo_config.ANGLE_MIN = 0
    servo_config.ANGLE_MAX = 180
    from xr_servo import Servo

    servo = Servo()
    has_servo = True
except Exception:
    rospy.logerr("Could not load xr_servo:\n%s", traceback.format_exc())

try:
    from xr_ultrasonic import Ultrasonic

    ultrasonic = Ultrasonic()
except Exception:
    rospy.logerr("Could not load ultrasonic sensor:\n%s", traceback.format_exc())


def hardware_preflight_errors():
    """Return required hardware components that failed to initialize."""
    errors = []
    if not has_gpio or gpio is None:
        errors.append("xr_gpio/GPIO")
    if not has_servo or servo is None:
        errors.append("xr_servo/servo controller")
    if ultrasonic is None:
        errors.append("xr_ultrasonic/ultrasonic sensor")
    return errors


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, float(value)))


def load_servo_state():
    # S4 and S5 use the measured mechanical limits above. The final two
    # values are the last known safe pan/pitch estimates for physical
    # controller sockets 7 and 8.
    defaults = [30.0, 160.0, 97.0, 0.0, 80.0, 89.0]
    try:
        with open(STATE_PATH, "r", encoding="utf-8") as source:
            loaded = json.load(source)
        values = loaded.get("angles", defaults)
        if len(values) != len(SERVO_CHANNELS):
            raise ValueError("angles must contain six values")
        return [
            clamp(values[index], *SERVO_LIMITS[index])
            for index in range(len(values))
        ]
    except Exception as error:
        rospy.logwarn("Using built-in servo state: %s", error)
        return defaults


def save_servo_state():
    try:
        # STATE_PATH is a Docker bind-mounted file. Write it in place because
        # replacing a bind-mount target with os.replace can fail with EBUSY.
        with open(STATE_PATH, "w", encoding="utf-8") as target:
            json.dump({"angles": current_servo_angles}, target, indent=2)
            target.write("\n")
    except Exception as error:
        rospy.logwarn_throttle(5.0, "Cannot save servo state: %s", error)


state_lock = threading.Lock()
drive_enabled = False
servo_enabled = False
drive_rearm_ready = False
servo_rearm_ready = False
servo_target_received = False
servo_disarm_since = None
last_drive_enable = 0.0
last_servo_enable = 0.0
last_cmd_vel = 0.0
previous_pwm_left = 0.0
previous_pwm_right = 0.0
applied_pwm_left = 0.0
applied_pwm_right = 0.0
previous_angular = 0.0
pwm_publisher = None
status_publisher = None
servo_state_publisher = None
servo_armed_publisher = None
drive_armed_publisher = None

current_servo_angles = load_servo_state()
desired_servo_angles = list(current_servo_angles)


def publish_status(message):
    rospy.loginfo(message)
    if status_publisher is not None:
        status_publisher.publish(String(message))


def publish_servo_state(angles=None):
    if servo_state_publisher is None:
        return
    message = Float32MultiArray()
    message.data = [
        float(value)
        for value in (current_servo_angles if angles is None else angles)
    ]
    servo_state_publisher.publish(message)


def publish_servo_armed(armed=None):
    if servo_armed_publisher is None:
        return
    servo_armed_publisher.publish(
        Bool(servo_enabled if armed is None else bool(armed)))


def publish_drive_armed(armed=None):
    if drive_armed_publisher is None:
        return
    drive_armed_publisher.publish(
        Bool(drive_enabled if armed is None else bool(armed)))


def servo_state_heartbeat(_event):
    """Keep pose and both authoritative gate acknowledgements fresh."""
    with state_lock:
        state_snapshot = list(current_servo_angles)
        armed_snapshot = bool(servo_enabled)
        drive_snapshot = bool(drive_enabled)
    publish_servo_state(state_snapshot)
    publish_servo_armed(armed_snapshot)
    publish_drive_armed(drive_snapshot)


def hard_stop_motors():
    global previous_pwm_left, previous_pwm_right
    global applied_pwm_left, applied_pwm_right
    global previous_angular
    previous_pwm_left = 0.0
    previous_pwm_right = 0.0
    applied_pwm_left = 0.0
    applied_pwm_right = 0.0
    previous_angular = 0.0
    if not has_gpio:
        return
    gpio.digital_write(gpio.IN1, 0)
    gpio.digital_write(gpio.IN2, 0)
    gpio.digital_write(gpio.IN3, 0)
    gpio.digital_write(gpio.IN4, 0)
    gpio.ena_pwm(0)
    gpio.enb_pwm(0)
    if pwm_publisher is not None:
        pwm_publisher.publish(Vector3(0.0, 0.0, 0.0))


def move_toward(current, target, maximum_delta):
    difference = target - current
    if abs(difference) <= maximum_delta:
        return target
    return current + (maximum_delta if difference > 0.0 else -maximum_delta)


def set_motors_pwm(logical_left, logical_right):
    global previous_pwm_left, previous_pwm_right
    global applied_pwm_left, applied_pwm_right
    if not has_gpio:
        return

    # Positive logical PWM is forward on both sides of this live chassis.
    physical_left = clamp(logical_left, -MAX_MOTOR_PWM, MAX_MOTOR_PWM)
    physical_right = clamp(logical_right, -MAX_MOTOR_PWM, MAX_MOTOR_PWM)

    if abs(physical_left) < 0.5 and abs(physical_right) < 0.5:
        hard_stop_motors()
        return

    physical_left = move_toward(
        previous_pwm_left, physical_left, MAX_PWM_STEP)
    physical_right = move_toward(
        previous_pwm_right, physical_right, MAX_PWM_STEP)
    previous_pwm_left = physical_left
    previous_pwm_right = physical_right

    left_magnitude = abs(physical_left)
    right_magnitude = abs(physical_right)
    left_magnitude = (
        0.0 if left_magnitude <= MOTOR_DEAD_ZONE
        else max(left_magnitude, MIN_MOTOR_PWM)
    )
    right_magnitude = (
        0.0 if right_magnitude <= MOTOR_DEAD_ZONE
        else max(right_magnitude, MIN_MOTOR_PWM)
    )

    # This is the final hardware boundary. Unity normally applies the same
    # limit, but the Pi must remain safe if a different or malformed command
    # publisher requests more. Keep this immediately before the H-bridge
    # writes so smoothing and the breakaway floor cannot bypass the cap.
    left_magnitude = min(left_magnitude, MAX_MOTOR_PWM)
    right_magnitude = min(right_magnitude, MAX_MOTOR_PWM)

    gpio.ena_pwm(int(left_magnitude))
    gpio.enb_pwm(int(right_magnitude))

    gpio.digital_write(gpio.IN1, 1 if physical_left > 0.0 else 0)
    gpio.digital_write(gpio.IN2, 1 if physical_left < 0.0 else 0)
    gpio.digital_write(gpio.IN3, 1 if physical_right > 0.0 else 0)
    gpio.digital_write(gpio.IN4, 1 if physical_right < 0.0 else 0)

    # Report what is actually applied to the H-bridge after the dead-zone and
    # minimum-breakaway floor.  Publishing the pre-floor request made a
    # nominal 11 PWM command look slow in telemetry even though the motors
    # physically received 35 PWM, which invalidated safety caps and odometry.
    applied_pwm_left = math.copysign(left_magnitude, physical_left)
    applied_pwm_right = math.copysign(right_magnitude, physical_right)

    if pwm_publisher is not None:
        pwm_publisher.publish(
            Vector3(
                float(applied_pwm_left),
                float(applied_pwm_right),
                0.0,
            )
        )


def drive_enable_callback(message):
    global drive_enabled, drive_rearm_ready, last_drive_enable, last_cmd_vel
    now = time.monotonic()
    requested = bool(message.data)
    with state_lock:
        if not requested:
            drive_enabled = False
            drive_rearm_ready = True
            last_drive_enable = now
        elif drive_enabled:
            last_drive_enable = now
        elif drive_rearm_ready:
            drive_enabled = True
            drive_rearm_ready = False
            last_drive_enable = now
            last_cmd_vel = now
        permitted = drive_enabled
    if not permitted:
        hard_stop_motors()
    publish_drive_armed(permitted)


def local_translation_block_reason(linear, now):
    """Return why a translating command is unsafe, or ``None`` if allowed.

    Callers hold ``state_lock`` so a sensor snapshot cannot be mixed across
    samples.  A pure angular command deliberately bypasses this translation
    gate; the existing drive heartbeat and command watchdogs still apply.
    """
    if abs(linear) < 1e-9:
        return None
    if (
        last_local_sensor_update <= 0.0
        or now - last_local_sensor_update > LOCAL_SENSOR_TIMEOUT
    ):
        return "local obstacle sensors stale"
    if linear > 0.0:
        if local_ultrasonic_metres <= MIN_FORWARD_CLEARANCE_METRES:
            return "local ultrasonic obstacle"
        if local_left_ir or local_right_ir:
            return "local front IR obstacle"
    elif local_rear_ir:
        return "local rear IR obstacle"
    return None


def velocity_callback(message):
    global drive_enabled, drive_rearm_ready
    global last_cmd_vel, previous_angular
    now = time.monotonic()
    try:
        raw_linear = float(message.linear.x)
        raw_angular = float(message.angular.z)
    except (AttributeError, TypeError, ValueError):
        raw_linear = float("nan")
        raw_angular = float("nan")
    if not math.isfinite(raw_linear) or not math.isfinite(raw_angular):
        with state_lock:
            drive_enabled = False
            drive_rearm_ready = False
        hard_stop_motors()
        publish_drive_armed(False)
        rospy.logwarn_throttle(
            1.0,
            "Rejected non-finite cmd_vel; drive disarmed and requires false before rearm",
        )
        return
    linear = clamp(raw_linear, -MAX_LINEAR, MAX_LINEAR)
    with state_lock:
        last_cmd_vel = now
        permitted = (
            drive_enabled
            and now - last_drive_enable <= DRIVE_ENABLE_TIMEOUT
        )
        translation_block_reason = (
            local_translation_block_reason(linear, now)
            if permitted
            else None
        )
    if not permitted:
        hard_stop_motors()
        return
    if translation_block_reason is not None:
        hard_stop_motors()
        rospy.logwarn_throttle(
            1.0,
            "Drive translation blocked by Pi safety gate: %s",
            translation_block_reason,
        )
        return

    previous_angular = 0.4 * raw_angular + 0.6 * previous_angular
    left_speed = linear + previous_angular * TURN_K
    right_speed = linear - previous_angular * TURN_K
    set_motors_pwm(
        left_speed * PWM_CONVERSION_FACTOR,
        right_speed * PWM_CONVERSION_FACTOR,
    )


def servo_enable_callback(message):
    global servo_enabled, servo_rearm_ready
    global servo_disarm_since, last_servo_enable, servo_target_received
    now = time.monotonic()
    requested = bool(message.data)
    enabled_now = False
    disabled_now = False
    ignored_rearm = False
    with state_lock:
        if not requested:
            disabled_now = servo_enabled
            servo_enabled = False
            if disabled_now:
                # Never carry an old six-servo target across a completed
                # hardware-gate session. A newly armed publisher must provide
                # a fresh complete packet before servo_timer can write.
                servo_target_received = False
            if servo_disarm_since is None:
                servo_disarm_since = now
            servo_rearm_ready = (
                now - servo_disarm_since >= SERVO_REARM_DISARM_DWELL
            )
            last_servo_enable = now
        elif servo_enabled:
            last_servo_enable = now
        elif servo_rearm_ready:
            servo_enabled = True
            servo_rearm_ready = False
            servo_disarm_since = None
            last_servo_enable = now
            enabled_now = True
        else:
            # A True received before the full disarmed dwell invalidates the
            # partial arm sequence. This prevents two conflicting publishers
            # (one stale False, one stale True) from alternating their way
            # into an armed hardware gate after a reconnect or Pi reboot.
            servo_disarm_since = None
            servo_rearm_ready = False
            ignored_rearm = True
        armed_now = servo_enabled
        target_ready = servo_target_received
    # This is the authoritative Pi-side acknowledgement. Publish it for every
    # request, including an ignored True, so Unity cannot confuse its local
    # intent with the hardware gate's actual state.
    publish_servo_armed(armed_now)
    if enabled_now:
        publish_status(
            "Absolute servo control enabled; latest complete target is ready."
            if target_ready
            else "Absolute servo control enabled; waiting for a complete target."
        )
    elif disabled_now:
        publish_status("Servo control disabled; holding current pose.")
    elif ignored_rearm:
        rospy.logwarn_throttle(
            2.0,
            "Servo enable ignored: send false before rearming.",
        )


def servo_targets_callback(message):
    global servo_target_received
    values = list(message.data)
    if len(values) != len(SERVO_CHANNELS):
        rospy.logwarn_throttle(
            2.0,
            "Absolute servo target must contain exactly six values",
        )
        return
    if not all(math.isfinite(float(value)) for value in values):
        rospy.logwarn_throttle(
            2.0,
            "Rejected non-finite absolute servo target",
        )
        return

    with state_lock:
        # Store the newest complete target even while disarmed. Physical
        # writes remain gated in servo_timer. This removes the cross-topic race
        # where a target arriving just before its enable message was discarded.
        for index, requested in enumerate(values):
            desired_servo_angles[index] = clamp(
                requested, *SERVO_LIMITS[index])
        servo_target_received = True


def safety_timer(_event):
    global drive_enabled, servo_enabled
    global drive_rearm_ready, servo_rearm_ready, servo_target_received
    global servo_disarm_since
    now = time.monotonic()
    with state_lock:
        drive_timed_out = drive_enabled and (
            now - last_drive_enable > DRIVE_ENABLE_TIMEOUT
            or now - last_cmd_vel > COMMAND_TIMEOUT
        )
        servo_timed_out = servo_enabled and (
            now - last_servo_enable > SERVO_ENABLE_TIMEOUT
        )
        if drive_timed_out:
            drive_enabled = False
            drive_rearm_ready = False
        if servo_timed_out:
            servo_enabled = False
            servo_rearm_ready = False
            servo_disarm_since = None
            servo_target_received = False
    if drive_timed_out and (
        abs(previous_pwm_left) > 0.0 or abs(previous_pwm_right) > 0.0
    ):
        hard_stop_motors()
    if drive_timed_out:
        publish_drive_armed(False)
    if servo_timed_out:
        publish_servo_armed(False)
        publish_status(
            "Servo heartbeat timed out; physical writes disabled. "
            "Send false before rearming."
        )
        return


def servo_timer(_event):
    if not has_servo:
        return
    now = time.monotonic()
    changed = False
    state_snapshot = None
    with state_lock:
        permitted = (
            servo_enabled
            and now - last_servo_enable <= SERVO_ENABLE_TIMEOUT
            and servo_target_received
        )
        if not permitted:
            return

        for index, channel in enumerate(SERVO_CHANNELS):
            next_angle = move_toward(
                current_servo_angles[index],
                desired_servo_angles[index],
                MAX_SERVO_STEPS[index],
            )
            next_angle = clamp(next_angle, *SERVO_LIMITS[index])
            if abs(next_angle - current_servo_angles[index]) < 0.5:
                continue
            servo.set(channel, int(round(next_angle)))
            current_servo_angles[index] = next_angle
            changed = True
        if changed:
            state_snapshot = list(current_servo_angles)
    if changed:
        save_servo_state()
        publish_servo_state(state_snapshot)


def read_ultrasonic_cm():
    """Read HC-SR04 with bounded waits so a missing echo cannot freeze ROS."""
    gpio.digital_write(gpio.TRIG, True)
    time.sleep(0.000015)
    gpio.digital_write(gpio.TRIG, False)

    deadline = time.monotonic() + 0.03
    while not gpio.digital_read(gpio.ECHO):
        if time.monotonic() >= deadline:
            return None

    pulse_start = time.monotonic()
    deadline = pulse_start + 0.03
    while gpio.digital_read(gpio.ECHO):
        if time.monotonic() >= deadline:
            return None

    distance_cm = (time.monotonic() - pulse_start) * 17000.0
    return distance_cm if 0.0 < distance_cm <= 500.0 else None


def sensor_timer(_event):
    """Publish filtered physical sensors using the legacy Unity wire format."""
    global last_ultrasonic_valid, last_local_sensor_update
    global local_ultrasonic_metres, local_left_ir, local_right_ir, local_rear_ir
    if not has_gpio or sensor_publisher is None:
        return
    try:
        now = time.monotonic()
        distance_cm = read_ultrasonic_cm()
        if distance_cm is not None:
            ultrasonic_history.pop(0)
            ultrasonic_history.append(distance_cm)
            last_ultrasonic_valid = now

        # User-verified custom connector layout: IO2 is physical left,
        # IO1 is physical right, IO3 is the claw sensor, and IO4 is rear.
        left = 1 if gpio.digital_read(gpio.IRF_R) == 0 else 0
        right = 1 if gpio.digital_read(gpio.IRF_L) == 0 else 0
        gripper = 1 if gpio.digital_read(GRIPPER_IR_PIN) == 0 else 0
        rear = 1 if gpio.digital_read(REAR_IR_PIN) == 0 else 0
        for history, value in (
            (left_ir_history, left),
            (right_ir_history, right),
            (gripper_ir_history, gripper),
            (rear_ir_history, rear),
        ):
            history.pop(0)
            history.append(value)

        filtered_gripper = 1 if sum(gripper_ir_history) >= 2 else 0
        # A missing echo must become a near/blocked fail-safe value instead of
        # keeping an old clear distance indefinitely.
        ultrasonic_metres = (
            float(sorted(ultrasonic_history)[1]) / 100.0
            if now - last_ultrasonic_valid <= 0.40
            else 0.0
        )
        filtered_left = 1 if sum(left_ir_history) >= 2 else 0
        filtered_right = 1 if sum(right_ir_history) >= 2 else 0
        filtered_rear = 1 if sum(rear_ir_history) >= 2 else 0
        with state_lock:
            local_ultrasonic_metres = ultrasonic_metres
            local_left_ir = filtered_left
            local_right_ir = filtered_right
            local_rear_ir = filtered_rear
            last_local_sensor_update = now
        sensor_publisher.publish(Quaternion(
            ultrasonic_metres,
            float(filtered_left),
            float(filtered_right),
            float(filtered_gripper),
        ))
        gripper_ir_publisher.publish(Int32(filtered_gripper))
        rear_ir_publisher.publish(Int32(filtered_rear))
    except Exception as error:
        rospy.logwarn_throttle(5.0, "Sensor read failed: %s", error)


def shutdown():
    global drive_enabled, drive_rearm_ready
    global servo_enabled, servo_rearm_ready, servo_disarm_since
    with state_lock:
        drive_enabled = False
        drive_rearm_ready = False
        servo_enabled = False
        servo_rearm_ready = False
        servo_disarm_since = None
    publish_servo_armed(False)
    publish_drive_armed(False)
    hard_stop_motors()
    save_servo_state()


def main():
    global pwm_publisher, status_publisher, servo_state_publisher
    global servo_armed_publisher, drive_armed_publisher
    global sensor_publisher, gripper_ir_publisher, rear_ir_publisher
    global SERVO_ENABLE_TIMEOUT, SERVO_REARM_DISARM_DWELL
    global CMD_VEL_TOPIC, DRIVE_ENABLE_TOPIC, SERVO_ENABLE_TOPIC, SERVO_TOPIC
    rospy.init_node("gfsx_unity_safe_bridge", anonymous=False)
    hardware_errors = hardware_preflight_errors()
    if hardware_errors:
        hard_stop_motors()
        raise RuntimeError(
            "Required GFS-X hardware initialization failed: "
            + ", ".join(hardware_errors)
        )
    default_prefix = os.environ.get("GFSX_COMMAND_PREFIX", "/gfsx")
    command_prefix = str(
        rospy.get_param("~command_prefix", default_prefix)
    ).strip().rstrip("/")
    if not command_prefix.startswith("/") or " " in command_prefix:
        raise ValueError("command_prefix must be an absolute ROS namespace")
    CMD_VEL_TOPIC = command_prefix + "/cmd_vel"
    DRIVE_ENABLE_TOPIC = command_prefix + "/drive_enable"
    SERVO_ENABLE_TOPIC = command_prefix + "/servo_enable"
    SERVO_TOPIC = command_prefix + "/servo_targets_degrees"
    SERVO_ENABLE_TIMEOUT = clamp(
        rospy.get_param(
            "~servo_enable_timeout",
            DEFAULT_SERVO_ENABLE_TIMEOUT,
        ),
        0.5,
        10.0,
    )
    SERVO_REARM_DISARM_DWELL = clamp(
        rospy.get_param(
            "~servo_rearm_disarm_dwell",
            DEFAULT_SERVO_REARM_DISARM_DWELL,
        ),
        0.5,
        5.0,
    )
    pwm_publisher = rospy.Publisher("/gfsx/motor_pwm", Vector3, queue_size=2)
    status_publisher = rospy.Publisher(
        "/gfsx/hardware_status", String, queue_size=5, latch=True)
    servo_state_publisher = rospy.Publisher(
        SERVO_STATE_TOPIC,
        Float32MultiArray,
        queue_size=2,
        latch=True,
    )
    servo_armed_publisher = rospy.Publisher(
        SERVO_ARMED_TOPIC,
        Bool,
        queue_size=2,
        latch=True,
    )
    drive_armed_publisher = rospy.Publisher(
        DRIVE_ARMED_TOPIC,
        Bool,
        queue_size=2,
        latch=True,
    )
    sensor_publisher = rospy.Publisher(
        "/sensor/data", Quaternion, queue_size=2)
    gripper_ir_publisher = rospy.Publisher(
        "/sensor/gripper_ir", Int32, queue_size=2)
    rear_ir_publisher = rospy.Publisher(
        "/sensor/rear_ir", Int32, queue_size=2)

    rospy.Subscriber(DRIVE_ENABLE_TOPIC, Bool, drive_enable_callback, queue_size=2)
    rospy.Subscriber(CMD_VEL_TOPIC, Twist, velocity_callback, queue_size=2)
    rospy.Subscriber(SERVO_ENABLE_TOPIC, Bool, servo_enable_callback, queue_size=2)
    rospy.Subscriber(
        SERVO_TOPIC, Float32MultiArray, servo_targets_callback, queue_size=2)

    rospy.Timer(rospy.Duration(0.05), safety_timer)
    rospy.Timer(rospy.Duration(0.05), servo_timer)
    rospy.Timer(rospy.Duration(0.5), servo_state_heartbeat)
    rospy.Timer(rospy.Duration(0.1), sensor_timer)
    rospy.on_shutdown(shutdown)
    hard_stop_motors()
    publish_status(
        "GFS-X safe bridge ready: motors stopped, real servos untouched; "
        "command namespace=" + command_prefix)
    publish_servo_armed(False)
    publish_drive_armed(False)
    publish_servo_state()
    rospy.spin()


atexit.register(shutdown)

if __name__ == "__main__":
    main()
