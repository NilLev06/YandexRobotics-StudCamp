#!/usr/bin/env python3
"""Coordinate autonomous object search from HTTP UI and voice commands."""

from __future__ import annotations

import json
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

ROVER_M2M_DIR = Path("/src/rover_m2m")
if str(ROVER_M2M_DIR) not in sys.path:
    sys.path.insert(0, str(ROVER_M2M_DIR))

from camera_snapshot import CameraSnapshotError, capture_jpeg  # noqa: E402

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


VOICE_COMMAND_RE = re.compile(
    r"\bровер\b[\s,!.:;\-]*.*?\bнайди\b[\s,!.:;\-]*.*?\b(мячик|мяч|кубик|куб)\b",
    re.IGNORECASE,
)
TARGETS = {"мяч": "мяч", "мячик": "мяч", "куб": "кубик", "кубик": "кубик"}
SEARCH_SCRIPT = "/src/rover_m2m/autonomous_object_search.py"
SEARCH_LOG = "/src/rover_m2m/object-search.log"
YANDEX_ENV_FILE = os.getenv(
    "YANDEX_CLOUD_ENV_FILE", "/src/rover_m2m/yandex-cloud.env"
)


def parse_voice_command(text: str) -> str | None:
    match = VOICE_COMMAND_RE.search(text.casefold())
    return TARGETS.get(match.group(1)) if match else None


class SearchManager(Node):
    def __init__(self) -> None:
        super().__init__("object_search_manager")
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.voice_pub = self.create_publisher(String, "/voice/speak", 10)
        self.status_pub = self.create_publisher(
            String, "/object_search/manager_status", transient
        )
        self.create_subscription(
            String, "/voice/recognized_text", self.voice_callback, 10
        )
        self.create_subscription(
            String, "/object_search/status", self.child_status_callback, 10
        )
        self.create_subscription(
            String, "/object_search/result", self.result_callback, transient
        )
        self.create_subscription(
            Bool, "/hardware/low_battery", self.battery_callback, transient
        )

        self.commands: queue.Queue[tuple[str, str | None]] = queue.Queue()
        self.process: subprocess.Popen[bytes] | None = None
        self.log_file: Any = None
        self.low_battery = False
        self.state = "IDLE"
        self.target: str | None = None
        self.message = "Готов к поиску"
        self.child_status: dict[str, Any] | None = None
        self.last_result: dict[str, Any] | None = None
        self.result_received = False
        self.stop_deadline: float | None = None
        self.camera_lock = threading.Lock()

        self.create_timer(0.2, self.tick)
        self.server = ThreadingHTTPServer(("0.0.0.0", 8091), self.make_handler())
        self.server_thread = threading.Thread(
            target=self.server.serve_forever, name="search-http", daemon=True
        )
        self.server_thread.start()
        self.publish_status()
        self.get_logger().info("Object search manager listening on port 8091")

    def make_handler(self) -> type[BaseHTTPRequestHandler]:
        manager = self

        class Handler(BaseHTTPRequestHandler):
            def send_json(self, code: int, payload: dict[str, Any]) -> None:
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def send_jpeg(self, body: bytes) -> None:
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store, max-age=0")
                self.send_header("X-Camera-Timestamp", str(time.time()))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802
                path = urlsplit(self.path).path
                if path == "/api/search/status":
                    self.send_json(200, manager.status_dict())
                elif path == "/api/camera/snapshot.jpg":
                    try:
                        with manager.camera_lock:
                            image = capture_jpeg()
                    except CameraSnapshotError as exc:
                        manager.get_logger().error(f"Camera snapshot failed: {exc}")
                        self.send_json(
                            503,
                            {"error": "camera_unavailable", "detail": str(exc)},
                        )
                        return
                    self.send_jpeg(image)
                else:
                    self.send_json(404, {"error": "not_found"})

            def do_POST(self) -> None:  # noqa: N802
                if self.path == "/api/search/stop":
                    manager.commands.put(("stop", None))
                    self.send_json(202, {"message": "Остановка принята"})
                    return
                if self.path != "/api/search/start":
                    self.send_json(404, {"error": "not_found"})
                    return
                try:
                    length = min(int(self.headers.get("Content-Length", "0")), 4096)
                    body = json.loads(self.rfile.read(length) or b"{}")
                except (ValueError, json.JSONDecodeError):
                    self.send_json(400, {"error": "invalid_json"})
                    return
                target = TARGETS.get(str(body.get("target", "")).casefold())
                if target is None:
                    self.send_json(400, {"error": "target_must_be_ball_or_cube"})
                    return
                if manager.low_battery:
                    self.send_json(409, {"error": "low_battery"})
                    return
                if manager.is_active():
                    self.send_json(409, {"error": "search_already_running"})
                    return
                manager.commands.put(("start", target))
                self.send_json(202, {"message": f"Запуск поиска: {target}"})

            def log_message(self, _format: str, *_args: Any) -> None:
                return

        return Handler

    def is_active(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def status_dict(self) -> dict[str, Any]:
        return {
            "state": self.state,
            "active": self.is_active(),
            "target": self.target,
            "message": self.message,
            "low_battery": self.low_battery,
            "child_status": self.child_status,
            "last_result": self.last_result,
        }

    def publish_status(self) -> None:
        self.status_pub.publish(
            String(data=json.dumps(self.status_dict(), ensure_ascii=False))
        )

    def speak(self, text: str) -> None:
        self.voice_pub.publish(String(data=text))

    def voice_callback(self, message: String) -> None:
        target = parse_voice_command(message.data)
        if target is None:
            return
        self.get_logger().info(f"Voice search request: {target}")
        if self.low_battery:
            self.speak("Низкий заряд аккумулятора. Замените аккумулятор перед поиском.")
        elif self.is_active():
            self.speak("Поиск уже выполняется.")
        else:
            self.commands.put(("start", target))

    def battery_callback(self, message: Bool) -> None:
        was_low = self.low_battery
        self.low_battery = bool(message.data)
        if self.low_battery and self.is_active():
            self.stop_search("Поиск остановлен: низкое напряжение аккумулятора")
            self.state = "LOW_BATTERY"
            self.speak("Низкое напряжение аккумулятора. Поиск остановлен. Замените аккумулятор.")
        elif self.low_battery and not was_low:
            self.state = "LOW_BATTERY"
            self.message = "Замените аккумулятор перед запуском поиска"
        elif was_low and not self.low_battery and not self.is_active():
            self.state = "IDLE"
            self.message = "Аккумулятор в норме, готов к поиску"
        self.publish_status()

    def child_status_callback(self, message: String) -> None:
        if not self.is_active():
            return
        try:
            self.child_status = json.loads(message.data)
        except json.JSONDecodeError:
            self.child_status = {"state": message.data}
        child_state = str(self.child_status.get("state", "searching"))
        self.message = f"Поиск «{self.target}»: {child_state}"
        self.publish_status()

    def result_callback(self, message: String) -> None:
        if not self.is_active() or self.result_received:
            return
        try:
            result = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if result.get("target") != self.target:
            return
        self.last_result = result
        self.result_received = True
        result_state = result.get("state")
        if result_state == "found":
            self.state = "FOUND"
            self.message = f"Объект «{self.target}» найден"
            self.speak(f"{self.target.capitalize()} найден.")
        elif result_state == "not_found":
            self.state = "NOT_FOUND"
            self.message = f"Объект «{self.target}» не найден"
            self.speak(f"Не удалось найти {self.target}.")
        elif result_state == "error":
            self.state = "ERROR"
            if result.get("reason") == "navigation_unavailable":
                self.message = "Поиск остановлен: навигация временно недоступна"
                self.speak("Не удаётся построить движение. Поиск остановлен.")
            else:
                self.message = "Поиск завершился с ошибкой"
        elif result.get("reason") == "low_battery":
            self.state = "LOW_BATTERY"
            self.message = "Поиск отменён из-за низкого напряжения"
        else:
            self.state = "STOPPED"
            self.message = "Поиск отменён"
        self.publish_status()

    def start_search(self, target: str) -> None:
        if self.is_active() or self.low_battery:
            return
        Path(SEARCH_LOG).parent.mkdir(parents=True, exist_ok=True)
        self.log_file = open(SEARCH_LOG, "ab", buffering=0)
        command = [
            "/usr/bin/python3",
            "-u",
            SEARCH_SCRIPT,
            target,
            "--env-file",
            YANDEX_ENV_FILE,
            "--max-waypoints",
            "60",
            "--max-duration",
            "1200",
            "--frontier-every",
            "3",
            "--sectors",
            "6",
        ]
        self.process = subprocess.Popen(
            command,
            stdout=self.log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.target = target
        self.state = "RUNNING"
        self.message = f"Ищу объект «{target}»"
        self.child_status = None
        self.last_result = None
        self.result_received = False
        self.stop_deadline = None
        self.speak(f"Начинаю поиск: {target}.")
        self.get_logger().info(f"Started object search pid={self.process.pid}: {target}")
        self.publish_status()

    def stop_search(self, reason: str = "Поиск остановлен") -> None:
        if self.is_active() and self.process is not None:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.stop_deadline = time.monotonic() + 8.0
            except ProcessLookupError:
                pass
        self.state = "STOPPED"
        self.message = reason
        self.publish_status()

    def close_process(self) -> None:
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
        self.process = None
        self.stop_deadline = None

    def tick(self) -> None:
        while True:
            try:
                command, target = self.commands.get_nowait()
            except queue.Empty:
                break
            if command == "start" and target is not None:
                self.start_search(target)
            elif command == "stop":
                if self.is_active():
                    self.stop_search()
                    self.speak("Поиск остановлен.")

        if self.process is None:
            return
        if (
            self.stop_deadline is not None
            and time.monotonic() >= self.stop_deadline
            and self.is_active()
        ):
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.message = "Поиск принудительно остановлен"
            except ProcessLookupError:
                pass
        return_code = self.process.poll()
        if return_code is None:
            return
        self.close_process()
        if not self.result_received and self.state == "RUNNING":
            self.state = "ERROR" if return_code else "NOT_FOUND"
            self.message = (
                f"Поиск завершился с ошибкой ({return_code})"
                if return_code
                else f"Объект «{self.target}» не найден"
            )
        self.publish_status()

    def destroy_node(self) -> bool:
        self.server.shutdown()
        self.stop_search("Менеджер поиска остановлен")
        if self.process is not None:
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
            self.close_process()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SearchManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
