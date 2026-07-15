import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from serial import Serial
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Imu
from threading import Thread
from hwnode.crc8 import crc8
from dataclasses import dataclass
import math
import struct
from typing import ClassVar

WHEEL_RADIUS = 0.021  # m
WHEEL_BASE = 0.128  # m
MAX_LINEAR = 0.3  # m/s
MAX_ANGULAR = 2.5  # rad/s
MAX_ACCEL_LINEAR = 0.1  # m/s²
MAX_ACCEL_ANGULAR = 5.0  # rad/s²
MAX_DECEL_LINEAR = 2.0  # m/s²
MAX_DECEL_ANGULAR = 5.0  # rad/s²
MAX_WHEEL_SPEED = 35.0  # rad/s

CONTROL_HZ = 30
ANGULAR_GAIN = 1.8
LINEAR_GAIN = 2.0

MIN_INPLACE_ANGULAR = 0.8
INPLACE_ANGULAR_THRESHOLD = 0.1
INPLACE_LINEAR_THRESHOLD = 0.05

PORT = "/dev/ttyAMA0"
SPEED = 115200

@dataclass
class Packet:
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    mag_x: float
    mag_y: float
    mag_z: float
    left_angle: float
    right_angle: float
    left_speed: float
    right_speed: float
    voltage: float
    current: float

    _sep: ClassVar[int] = 0x7E
    _fmt: ClassVar[str] = "<hhhhhhhhhffffBBHx"
    _size:  ClassVar[int] = struct.calcsize(_fmt)
    _payload_size: ClassVar[int] = 36

    @staticmethod
    def crc16_xmodem(data: bytes, init: int = 0x0000) -> int:
        crc = init
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    @classmethod
    def unpack(cls, buff: bytes):
        if len(buff) != cls._size: return
        values = struct.unpack(cls._fmt, buff)
        if cls.crc16_xmodem(buff[:cls._payload_size]) != values[15]:
            return
        return cls(
            accel_x=values[0],
            accel_y=values[1],
            accel_z=values[2],
            gyro_x=values[3] * 250.0 / 32768.0 / 180 * math.pi,
            gyro_y=values[4] * 250.0 / 32768.0 / 180 * math.pi,
            gyro_z=values[5] * 250.0 / 32768.0 / 180 * math.pi,
            mag_x=values[6],
            mag_y=values[7],
            mag_z=values[8],
            left_angle=values[9],
            right_angle=values[10],
            left_speed=values[11],
            right_speed=values[12],
            voltage=values[13] / 10,
            current=values[14] / 10,
        )

class HardwareNode(Node):
    def __init__(self):
        super().__init__("hwnode")

        self.L = WHEEL_BASE
        self.R = WHEEL_RADIUS
        self.max_wheel = MAX_WHEEL_SPEED * self.R

        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.v_left = 0.0
        self.v_right = 0.0
        self.battery_v = None

        self.last_packet: Packet = None
        self.last_cmd_time = self.get_clock().now()
        self.ser = Serial(port=PORT, baudrate=SPEED, timeout=0.1, exclusive=True)
        self.get_logger().info("Serial подключен")

        self.control_timer = self.create_timer(1.0 / CONTROL_HZ, self.update)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 1)
        self.imu_pub = self.create_publisher(Imu, "/hardware/imu", 1)
        self.status_pub = self.create_publisher(Float32MultiArray, "/hardware/status", 1)
        self.odom_pub = self.create_publisher(Odometry, "/hardware/odom", 1)
        self.battery_pub = self.create_publisher(Float32, "/hardware/battery", 1)

        self.read_thread = Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def read_loop(self):
        buff = b""

        while rclpy.ok():
            chunk = self.ser.read_until(b"\x7E")
            if len(buff) + len(chunk) > Packet._size: buff = b""
            buff += chunk
            # chunk = chunk.rstrip(b"\x7E")
            packet = Packet.unpack(buff)
            if packet is None: continue
            self.last_packet = packet

            now = self.get_clock().now().to_msg()

            status = Float32MultiArray()
            status.data = [
                self.v_left,
                packet.left_speed,
                packet.left_angle,
                self.v_right,
                packet.right_speed,
                packet.right_angle,
            ]
            self.status_pub.publish(status)

            batt = Float32()
            if self.battery_v is None:
                self.battery_v = packet.voltage
            else:
                self.battery_v = 0.999 * self.battery_v + 0.001 * packet.voltage
            batt.data = self.battery_v
            self.battery_pub.publish(batt)

            imu = Imu()
            imu.header.frame_id = "base_link"
            imu.header.stamp = now
            imu.orientation_covariance = [-1.0] + [0.0] * 8
            imu.linear_acceleration_covariance = [-1.0] + [0.0] * 8
            imu.angular_velocity.x = packet.gyro_x
            imu.angular_velocity.y = packet.gyro_y
            imu.angular_velocity.z = packet.gyro_z
            imu.angular_velocity_covariance = [0.0] * 9
            imu.angular_velocity_covariance[0] = 0.001
            imu.angular_velocity_covariance[4] = 0.001
            imu.angular_velocity_covariance[8] = 0.001
            self.imu_pub.publish(imu)

            odom = Odometry()
            odom.child_frame_id = "base_link"
            odom.header.frame_id = "base_link"
            odom.header.stamp = now
            linear_vel = (packet.left_speed + packet.right_speed) * self.R / 2
            odom.twist.twist.linear.x = linear_vel
            odom.twist.covariance = [0.0] * 36
            odom.twist.covariance[0] = 0.001
            odom.twist.covariance[35] = 1000.0
            if abs(packet.left_speed) < 0.001 and abs(packet.right_speed) < 0.001:
                odom.twist.covariance[0] = 0.000001
                odom.twist.covariance[35] = 0.000001
            self.odom_pub.publish(odom)

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        v = msg.linear.x * LINEAR_GAIN
        w = msg.angular.z * ANGULAR_GAIN

        v = math.copysign(min(abs(v), MAX_LINEAR), v)
        w = math.copysign(min(abs(w), MAX_ANGULAR), w)

        if abs(v) < INPLACE_LINEAR_THRESHOLD and abs(w) > INPLACE_ANGULAR_THRESHOLD:
            w = math.copysign(max(abs(w), MIN_INPLACE_ANGULAR), w)
        elif abs(w) < INPLACE_ANGULAR_THRESHOLD:
            w = 0.0

        self.target_v, self.target_w = v, w
        print(self.target_v, self.target_w)

    def ramp(self, current, target, accel_step, decel_step):
        step = accel_step if abs(target) > abs(current) else decel_step
        if target > current:
            return min(current + step, target)
        return max(current - step, target)

    def update(self):
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > 0.3:
            self.target_v = 0.0
            self.target_w = 0.0

        self.current_v = self.ramp(
            self.current_v,
            self.target_v,
            MAX_ACCEL_LINEAR / CONTROL_HZ,
            MAX_DECEL_LINEAR / CONTROL_HZ,
        )
        self.current_w = self.ramp(
            self.current_w,
            self.target_w,
            MAX_ACCEL_ANGULAR / CONTROL_HZ,
            MAX_DECEL_ANGULAR / CONTROL_HZ,
        )

        v_left = (self.current_v - self.current_w * self.L / 2) / self.R
        v_right = (self.current_v + self.current_w * self.L / 2) / self.R
        peak = max(abs(v_left), abs(v_right), 1e-9)
        scale = min(1.0, MAX_WHEEL_SPEED / peak)
        self.v_left = v_left * scale
        self.v_right = v_right * scale

        buf = struct.pack("<ffB", self.v_left, self.v_right, 0)
        buf = b"\xA0" + buf
        crc = crc8()
        crc.update(buf)
        buf = b"\x7E" + buf + crc.digest()
        # debug = [f"{i:02x}" for i in buf]
        # print(f"-> {' '.join(debug)}")
        self.ser.write(buf)
        self.ser.flush()


def main():
    rclpy.init()
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
