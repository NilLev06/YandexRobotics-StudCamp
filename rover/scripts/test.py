#!/usr/bin/env python3

from pathlib import Path
import asyncio
import time
import abc
from typing import Callable, ClassVar
from nicegui import ui
from starlette.requests import Request
from dataclasses import dataclass
import traceback
from serial import Serial
import socket
import struct
import math
import json
import yaml
import re


class BaseTest(abc.ABC):
    name: ClassVar[str] = ...
    tests: ClassVar[dict] = {}

    def __init__(self):
        self.status = "NOT RUN"
        self.log_func: Callable[[str], None] = None
        self.output: ui.column = None
        self.run_btn: ui.button = None

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        BaseTest.tests[cls] = True

    def log(self, msg: str, color: str = "default"):
        COLORS = {
            "red": "\x1b[91m",
            "green": "\x1b[92m",
            "yellow": "\x1b[93m",
            "blue": "\x1b[94m",
            "reset": "\x1b[0m",
        }
        if color != "default":
            reset = COLORS["reset"]
            code = COLORS.get(color, reset)
            msg = f"{code}{msg}{reset}"
        self.log_func(msg)

    async def shell(self, cmd: str, check: bool = True, timeout: float = 10, **kwargs):
        self.log(f">>> $ {cmd}")
        proc = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )

        logs = ""
        async def read_stream(stream):
            nonlocal logs
            while True:
                line = await stream.readline()
                if not line: break
                line = line.decode().rstrip()
                logs += line + "\n"
                self.log(line)
        
        try:
            gather = asyncio.gather(read_stream(proc.stdout), read_stream(proc.stderr))
            await asyncio.wait_for(gather, timeout=timeout)
            await proc.wait()
        except asyncio.TimeoutError:
            proc.kill()
            await proc.wait()
            assert False, f"Command timed out after {timeout}s"

        assert not check or proc.returncode == 0, f"Bad return code: {proc.returncode}"
        return logs

    async def run(self):
        self.log(f"\r\n===== START: {self.name} =====\r\n", color="yellow")
        try:
            self.output.clear()
            self.output.set_visibility(False)
            self.run_btn.disable()
            self.status = "RUNNING"
            res = await self.test()
            self.status = "PASS" if res else "FAIL"
        except AssertionError as err:
            self.log(f"----- ASSERTION FAILED -----\n", color="red")
            self.log(str(err) + "\n")
            self.status = "FAIL"
        except Exception as err:
            self.log(f"----- UNHANDLED TEST ERROR -----\n", color="red")
            self.log(traceback.format_exc().replace("\n", "\n\r"))
            self.status = "FAIL"
        color = "green" if self.status == "PASS" else "red"
        self.log(f"\r\n===== [{self.status}] {self.name} =====", color=color)
        self.run_btn.enable()

    @abc.abstractmethod
    async def test(self):
        pass

class RaspberryTest(BaseTest):
    name = "RASPBERRY"

    async def test(self):
        logs = await self.shell("vcgencmd otp_dump | grep '^32:'")
        line = logs.splitlines()[0]
        assert line.startswith("32:00d")
        return True

class CameraTest(BaseTest):
    name = "CAMERA"

    async def test(self):
        await self.shell("docker stop ros", check=False, timeout=10)
        logs = await self.shell("rpicam-hello --list-cameras")
        lines = logs.splitlines()
        count = sum(1 for line in lines if line.strip() and line[0].isdigit())
        assert count == 1, f"Expected 1 camera, found {count}"

        output_file = Path("/tmp/camera-test.png")
        output_file.unlink(missing_ok=True)
        cmd = (
            "rpicam-still --timeout 1000 --encoding png "
            f"--width 1280 --height 720 --output {output_file}"
        )
        await self.shell(cmd)
        assert output_file.exists(), f"Output file missing: {output_file}"

        with self.output.classes(add="h-60 items-center"):
            self.output.set_visibility(True)
            img = ui.image(output_file).classes("max-h-full").props("fit=contain")
            img.force_reload()
        return True


class SpeakerMicTest(BaseTest):
    name = "SPEAKER + MIC"

    async def test(self):
        output_file = Path("/tmp/mic-test.wav")
        output_file.unlink(missing_ok=True)
        record_proc = self.shell(
            "arecord -D plughw:0 -c2 -r 48000 -f S32_LE -t wav "
            f"-V stereo -d 5 -v {output_file}"
        )
        speaker_proc = self.shell("sleep 2 && speaker-test -t wav -c 1 -l 2")
        await asyncio.gather(record_proc, speaker_proc)
        assert output_file.exists(), f"Output file missing: {output_file}"
        assert output_file.stat().st_size != 0, f"Output file empty: {output_file}"
        self.log("NOTICE: Check recorded audio manually", color="yellow")

        with self.output:
            self.output.set_visibility(True)
            ui.audio(output_file).classes("w-full")
        return True


class MCUTest(BaseTest):
    name = "STM COMM + IMU CHECK"

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
            values = struct.unpack(cls._fmt, buff)
            # if cls.crc16_xmodem(buff[:cls._payload_size]) != values[15]:
            #     raise RuntimeError("Kek2")
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

    def crc8(self, data: bytes, poly: int = 0x31, init: int = 0xFF) -> int:
        crc = init
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80: crc = (crc << 1) ^ poly
                else: crc <<= 1
                crc &= 0xFF
        return crc

    @staticmethod
    def imu_nonzero(pack: "MCUTest.Packet") -> bool:
        return any(
            abs(value) > 1e-3
            for value in (
                pack.accel_x,
                pack.accel_y,
                pack.accel_z,
                pack.gyro_x,
                pack.gyro_y,
                pack.gyro_z,
                pack.mag_x,
                pack.mag_y,
                pack.mag_z,
            )
        )

    def send_cmd(self, ser: Serial, left: float, right: float, log: bool = True):
        buf = struct.pack("<ffB", left, right, 0)
        buf = b"\xA0" + buf
        buf = b"\x7E" + buf + self.crc8(buf).to_bytes()
        if log: self.log(f">>> {':'.join(f'{x:02X}' for x in buf)}")
        ser.write(buf)
        ser.flush()

    async def read_serial(self, ser: Serial, timeout: float = 2.0, packets: list = None, logfmt: str = None, log_packets: bool = True):
        if logfmt is None:
            logfmt = (
                "gyro: [{0.gyro_x:.2f}, {0.gyro_y:.2f}, {0.gyro_z:.2f}] | "
                "mag: [{0.mag_x:.2f}, {0.mag_y:.2f}, {0.mag_z:.2f}]\r\n"
                "speed: [{0.left_speed:.2f}, {0.right_speed:.2f}] | "
                "angle: [{0.left_angle:.2f}, {0.right_angle:.2f}] | "
                "battery: [{0.voltage:.2f}, {0.current:.2f}]"
            )

        data = b""
        packets = [] if packets is None else packets
        start = time.perf_counter()

        while time.perf_counter() - start < timeout:
            if not ser.in_waiting:
                await asyncio.sleep(0.1)
                continue

            chunk = ser.read_until(b"\x7E")
            # self.log(f"RAW : {':'.join(f'{x:02X}' for x in chunk)}")
            data += chunk
            if len(data) >= self.Packet._size:
                
                last_sep_index = -1
                for i in range(len(data) - 1, -1, -1):
                    if data[i] == 0x7E:
                        last_sep_index = i
                        break
                # self.log(f"last sep: {last_sep_index}")
                if last_sep_index == -1: continue
                if last_sep_index < self.Packet._size: continue
                chunk = data[last_sep_index - self.Packet._size + 1:last_sep_index + 1]
                if log_packets: self.log(f"<<< {':'.join(f'{x:02X}' for x in chunk)}")
                if chunk.count(0x7E) > 1:
                    self.log("!!! WARNING: 0x7E inside packet - skipping")
                    continue
                pack = self.Packet.unpack(chunk)
                assert pack is not None, "Failed to parse packet"
                # self.log(f"unpack: {pack}")
                self.log(logfmt.format(pack))
                if pack is not None:
                    packets.append(pack)
                    data = data[last_sep_index + 1:-1]
            
            await asyncio.sleep(0)
        
        return packets

    async def open_serial(self):
        await self.shell("docker stop ros", check=False, timeout=10)
        ser = Serial("/dev/ttyAMA0", baudrate=115200, timeout=0.1, exclusive=True)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser

    async def test(self):
        ser = await self.open_serial()
        packets = await self.read_serial(ser)

        if not packets:
            msg = "No valid packets decoded"
            self.log(msg, color="red")
            return False

        imu_present = any(self.imu_nonzero(pack) for pack in packets)
        if not imu_present:
            self.log("IMU data not detected; skipping IMU validation", color="yellow")

        good_packet_count = 0
        for pack in packets:
            battery_ok = pack.voltage > 0
            imu_ok = True if not imu_present else self.imu_nonzero(pack)
            good_packet_count += battery_ok and imu_ok

        msg = f"Good IMU packets: {good_packet_count} out of {len(packets)} total"
        self.log(msg, color="yellow")
        assert good_packet_count > len(packets) / 2, "Broken feedback"

        return True


class MoveTest(MCUTest):
    name = "SIMPLE MOVE TEST"

    async def test(self):
        ser = await self.open_serial()
        packets = []

        SPEED = 10.0
        MOVES = [
            (+SPEED, +SPEED, 1.0),
            (-SPEED, +SPEED, 1.0),
            (+SPEED, +SPEED, 1.0),
            (+SPEED, -SPEED, 1.0),
        ]
        RESULTS = []

        async def move_func():
            for left, right, delay in MOVES:
                self.log(f"\r\n***** MOVE: [{left:5.1f}, {right:5.1f}] *****\r\n")
                self.send_cmd(ser, left, right)
                packet_index = len(packets) - 1
                await asyncio.sleep(delay)
                RESULTS.append(packets[packet_index:])

        try:
            read_func = self.read_serial(
                ser,
                packets=packets,
                timeout=sum(x[2] for x in MOVES),
                logfmt="speed: [{0.left_speed:6.1f}, {0.right_speed:6.1f}]",
                log_packets=False,
            )
            await asyncio.gather(move_func(), read_func)
            self.send_cmd(ser, 0.0, 0.0)

            self.log("\r\n")
            for (left, right, _), results in zip(MOVES, RESULTS):
                assert len(results), f"No packets received for move: [{left:.1f}, {right:.1f}]"
                left_err = [left - x.left_speed for x in results]
                left_rms = math.sqrt(sum(x ** 2 for x in left_err) / len(left_err))
                right_err = [right - x.right_speed for x in results]
                right_rms = math.sqrt(sum(x ** 2 for x in right_err) / len(right_err))
                self.log(
                    f"MOVE [{left:6.1f}, {right:6.1f}] "
                    f"SPEED RMS ERR: [{left_rms:.3f}, {right_rms:.3f}]"
                )
            self.log("\r\nNOTICE: Check speed RMS errors manually", color="yellow")

        finally:
            self.send_cmd(ser, 0.0, 0.0, log=False)

        return True

class StartDockerTest(BaseTest):
    name = "START DOCKER"

    async def test(self):
        Path("/home/robomarvel/.autostart").touch()
        await self.shell("docker start ros", timeout=10)
        return True


class DockerROSTest(BaseTest):
    name = "DOCKER + ROS"
    CONTAINER_NAME = "ros"

    EXPECTED_NODES = [
        "/hwnode",
        "/ld19_node",
        "/icp_odom",
        "/robot_state_publisher",
        "/foxglove_bridge",
        "/slam_toolbox",
        "/nav2_container",
        "/bt_navigator",
        "/behavior_server",
        "/controller_server",
        "/planner_server",
        "/global_costmap/global_costmap",
        "/local_costmap/local_costmap",
        "/lifecycle_manager_navigation",
    ]

    TOLERANCE = 0.9

    TRANSFORMS = [
        ("lidar", "base_link", -1),
        ("base_link", "odom", 10.0),
        ("odom", "map", 1.0),
    ]

    TOPICS = [
        ("/scan", 10.0, 3),
        # из за этого топика весь тест пишет fail, хотя почти на каждом роботе имушки просто нет, да и для нее есть отдельный тест
        # ("/hardware/imu", 50.0, 3),
        ("/hardware/odom", 50.0, 3),
        ("/icp/odom", 10.0, 3),
        ("/local_costmap/costmap", 1.5, 10),
        ("/global_costmap/costmap", 0.5, 10),
        ("/map", 0.2, 20),
    ]

    async def _docker_exec(self, cmd: str, check: bool = True, timeout: float = 10.0) -> str:
        cmd = f"docker exec {self.CONTAINER_NAME} bash -c 'export PS1=\"dummy\" && source ~/.bashrc && {cmd}'"
        return await self.shell(cmd, check=check, timeout=timeout)

    async def _start_container(self):
        Path("/home/robomarvel/.autostart").touch()
        await self.shell(f"docker stop {self.CONTAINER_NAME}", check=False, timeout=10)
        await self.shell(f"docker start {self.CONTAINER_NAME}", timeout=10)
        self.log("Waiting for ROS to initialize...", color="yellow")
        await asyncio.sleep(5)

    async def _check_nodes(self):
        self.log("\n--- Checking ROS nodes ---", color="yellow")
        output = await self._docker_exec("ros2 node list", timeout=10)
        nodes = set(line.strip() for line in output.splitlines() if line.strip())
        missing = set(self.EXPECTED_NODES) - nodes
        assert not missing, f"Missing nodes: {sorted(missing)}"
        self.log(f"All {len(self.EXPECTED_NODES)} expected nodes found", color="green")

    async def _check_camera_stream(self):
        self.log("\n--- Checking camera stream (mediamtx) ---", color="yellow")
        output = await self._docker_exec("ps -ef | grep '[m]ediamtx'", timeout=5, check=False)
        assert output.strip(), "Camera stream is not running (camera_stream.yaml)" 
        self.log("Camera stream process detected", color="green")

    async def _check_transforms(self):
        self.log("\n--- Checking transforms via view_frames ---", color="yellow")
        cmd = "ros2 run tf2_tools view_frames -o /tmp/frames --wait-time 5"
        output = await self._docker_exec(cmd, timeout=10)

        match = re.search(r'Result:tf2_msgs\.srv\.FrameGraph_Response\(frame_yaml="(.*)"\)', output, re.DOTALL)
        assert match, "Could not extract frame_yaml from view_frames output"
        data = match.group(1)
        data = data.replace('\\"', '"')
        data = data.replace("\\n", "\n")
        self.log(f"\r\nVIEW FRAMES RESULT:", color="yellow")
        self.log(data.replace("\n", "\r\n"))
        frames = yaml.safe_load(data)

        for child, parent, min_rate in self.TRANSFORMS:
            self.log(f"Checking transform: {child} -> {parent}")
            assert child in frames, f"Child frame '{child}' not found"
            info = frames[child]
            actual_parent = info.get("parent")
            assert actual_parent == parent, f"Transform {child} has parent '{actual_parent}', expected '{parent}'"
            if min_rate > 0:
                rate = info.get("rate")
                assert rate is not None, f"Rate not available for dynamic transform {child}"
                assert rate >= min_rate * self.TOLERANCE, f"Transform {child} rate too low: {rate:.3f} Hz < {min_rate} Hz"
                self.log(f"--> OK: rate = {rate:.3f} Hz (≥ {min_rate})", color="green")
            else:
                self.log(f"--> OK: static transform", color="green")

    async def _check_topic(self, topic: str, min_rate: float, timeout: float):
        self.log(f"\n--- Checking topic: {topic} ---", color="yellow")
        cmd = f"timeout {timeout} ros2 topic hz {topic}"
        output = await self._docker_exec(cmd, timeout=timeout + 1, check=False)
        avg_rate = None
        for line in output.splitlines():
            if "average rate" in line:
                match = re.search(r"average rate:\s+([0-9.]+)", line)
                if match:
                    avg_rate = float(match.group(1))
                    break
        if avg_rate is None or avg_rate == 0.0:
            if "no new messages" in output or not output.strip():
                assert False, f"No messages received"
            assert False, f"Could not determine message rate"
        assert avg_rate >= min_rate * self.TOLERANCE, f"Rate too low: {avg_rate:.1f} Hz < {min_rate} Hz"
        self.log(f"Topic rate OK: {avg_rate:.3f} >> {min_rate} Hz", color="green")

    async def test(self):
        await self._start_container()
        await self._check_nodes()
        await self._check_camera_stream()
        await self._check_transforms()
        for topic, min_rate, timeout in self.TOPICS:
            await self._check_topic(topic, min_rate, timeout)
        return True


class Nav2RoundTripTest(DockerROSTest):
    name = "NAV2 ROUND TRIP"

    async def _reset_map(self):
        self.log("\n--- Resetting map ---", color="yellow")
        services = await self._docker_exec("ros2 service list", timeout=10)
        reset_services = [line.strip() for line in services.splitlines() if "reset" in line.lower()]
        assert reset_services, "No reset service found"
        service = "/slam_toolbox/reset" if "/slam_toolbox/reset" in reset_services else reset_services[0]
        service_types = await self._docker_exec("ros2 service type " + service, timeout=10)
        service_type = service_types.strip().splitlines()[-1]
        await self._docker_exec(f"ros2 service call {service} {service_type} {{}}", timeout=10)

    async def _send_goal(self, x: float, y: float, yaw: float = 0.0, timeout: float = 45.0):
        yaw = float(yaw)
        self.log(f"\n--- Navigate to x={x:.2f}m, yaw={math.degrees(yaw):.1f}deg ---", color="yellow")
        half_yaw = yaw / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": float(f"{x:.3f}"), "y": float(f"{y:.3f}"), "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
                },
            }
        }
        goal_json = json.dumps(goal, separators=(",", ":"))
        goal_arg = goal_json.replace('"', '\\"')
        cmd = f"timeout {timeout:.0f} ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \"{goal_arg}\""
        output = await self._docker_exec(cmd, timeout=timeout + 2, check=False)
        assert "Goal accepted" in output, "NavigateToPose goal was not accepted"
        success_markers = (
            "Goal succeeded",
            "status: 4",
            "STATUS_SUCCEEDED",
            "Goal finished with status: SUCCEEDED",
        )
        assert any(marker in output for marker in success_markers), "NavigateToPose goal did not succeed"

    async def test(self):
        await self._start_container()
        await self._check_nodes()
        await self._check_transforms()
        await self._reset_map()
        await self._send_goal(0.5, 0.0, yaw=0.0)
        await self._send_goal(0.0, 0.5 yaw=0.0)
        await self._send_goal(0.5, 0.5 yaw=0.0)
        return True


# ----------------------------------------------------------------


class StatusLabel(ui.label):
    COLORS = {
        "running": "text-warning",
        "pass": "text-positive",
        "fail": "text-negative",
        "not run": "text-grey",
    }

    def _handle_text_change(self, text: str) -> None:
        super()._handle_text_change(text)
        self.classes(remove=(" ".join(self.COLORS.values())))
        res = self.COLORS.get(text.lower())
        if res is not None: self.classes(add=res)


INTRO = """
  ___ ___ __  __   _____   _____ _____ ___ __  __   _____ ___ ___ _____ ___ 
 │ _ ╲ _ )  ╲╱  │ ╱ __╲ ╲ ╱ ╱ __│_   _│ __│  ╲╱  │ │_   _│ __╱ __│_   _╱ __│
 │   ╱ _ ╲ │╲╱│ │ ╲__ ╲╲ V ╱╲__ ╲ │ │ │ _││ │╲╱│ │   │ │ │ _│╲__ ╲ │ │ ╲__ ╲
 │_│_╲___╱_│  │_│ │___╱ │_│ │___╱ │_│ │___│_│  │_│   │_│ │___│___╱ │_│ │___╱
                                                                            
"""

HOSTNAME = socket.gethostname()

test_classes = [RaspberryTest, CameraTest, SpeakerMicTest, MoveTest, MCUTest,
                Nav2RoundTripTest, StartDockerTest, DockerROSTest]

@ui.page("/tests", favicon="✅")
def main_page():
    ui.dark_mode(value=True)
    ui.query(".nicegui-content").classes("p-0")
    ui.page_title(f"{HOSTNAME.upper()} | TESTS")
    test_instances = []

    with ui.splitter(value=25).classes("w-full h-screen") as splitter:
        with splitter.before:
            with ui.column().classes("w-full p-4 overflow-auto"):
                with ui.row().classes("w-full items-center"):
                    ui.label("RBM SYSTEM TESTS").classes("text-h5 col")

                    async def run_all():
                        run_all_btn.disable()
                        for test in test_instances:
                            await test.run()
                        run_all_btn.enable()
                    run_all_btn = ui.button("RUN ALL", on_click=run_all, icon="play_arrow")

                ui.separator()

                for test_class in test_classes:
                    test = test_class()
                    test_instances.append(test)

                    with ui.row().classes("items-center gap-4 p-2 border rounded w-full").style("border-color: rgba(255, 255, 255, 0.25)"):
                        ui.label(test.name).classes("font-bold col")
                        StatusLabel().bind_text_from(test, "status")
                        btn = ui.button("RUN", on_click=test.run).props("outline")
                        output = ui.row().classes("w-full")
                        output.set_visibility(False)

                        test.log_func = lambda line: terminal.write(line + "\r\n")
                        test.output = output
                        test.run_btn = btn

        with splitter.after:
            with ui.column().classes("w-full h-full p-4"):
                terminal = ui.xterm().classes("size-full")
                ui.element("q-resize-observer").on("resize", terminal.fit)
                intro = INTRO + "—" * (len(INTRO.splitlines()[-1]) + 1) + "\n"
                intro = intro.replace("\n", "\r\n")
                terminal.write(intro)


# ----------------------------------------------------------------

LINKS = [
    ("Foxglove", "category", "https://foxglove-ssl.robotics-lab.ru/?ds=foxglove-websocket&ds.url=ws%3A%2F%2F{ip}%3A8765"),
    ("Jupyter Lab", "code", "http://{ip}:8080"),
    ("Terminal (host)", "terminal", "http://{ip}:8100"),
    ("Terminal (docker)", "terminal", "http://{ip}:8200"),
    ("System Tests", "checklist", "/tests"),
    ("Camera Stream", "videocam", "http://{ip}:8889/cam"),
]

@ui.page("/", favicon="🚀")
async def welcome_page(request: Request):
    ui.dark_mode(value=True)
    ui.query(".nicegui-content").classes("p-0")
    ui.page_title(f"{HOSTNAME.upper()} | HOME")
    ip = request.url.hostname

    with ui.column().classes("items-center justify-center w-full h-screen gap-2"):
        ui.label(f"{HOSTNAME.upper()}").classes("text-h2 font-mono")
        ui.label(f"IP: {ip}").classes("text-h6 font-mono mb-5")

        with ui.card().classes("w-100 p-6 shadow-lg"):
            for label, icon, url in LINKS:
                with ui.link(
                    target=url.format(ip=ip),
                    new_tab=True
                ).classes("bg-red-900 text-white no-underline rounded w-full p-2"):
                    with ui.row().classes("items-center justify-center"):
                        ui.icon(icon, size="md")
                        ui.label(label.upper())


ui.run(title="ROBOMARVEL", show=False, port=80, favicon="✅", reconnect_timeout=10)
