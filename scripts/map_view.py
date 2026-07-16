#!/usr/bin/env python3
"""Live map view: renders the SLAM occupancy grid with the robot's current
position, its starting ("home") position, and markers for objects the guarded
search behavior has confirmed (from /search/found), as a JPEG snapshot served
over HTTP. Read-only: this node never commands motion."""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlsplit

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

LOG = logging.getLogger("map-view")

MAX_MARKERS = 300
MARKER_STALE_S = 3600.0  # drop found-object markers older than this

PAGE = """<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RoboMarvel — карта</title>
<style>
  :root { color-scheme: dark; font-family: system-ui, sans-serif; }
  body { margin: 0; background: #0b0f14; color: #e8edf3; }
  header { display: flex; align-items: center; gap: 14px; padding: 10px 16px; border-bottom: 1px solid #1c2530; }
  header h1 { font-size: 16px; margin: 0; flex: 1; }
  .legend { display: flex; gap: 14px; font-size: 12px; color: #aeb8c4; }
  .legend span { display: inline-flex; align-items: center; gap: 5px; }
  .dot { width: 9px; height: 9px; border-radius: 50%; display: inline-block; }
  main { display: flex; justify-content: center; padding: 12px; }
  img { max-width: 100%; height: auto; background: #05070a; border-radius: 6px; }
  #status { font-size: 12px; color: #8793a1; padding: 0 16px 10px; }
</style>
</head>
<body>
<header>
  <h1>RoboMarvel &middot; карта</h1>
  <div class="legend">
    <span><span class="dot" style="background:#38d996"></span>ровер</span>
    <span><span class="dot" style="background:#7cc7ff"></span>старт</span>
    <span><span class="dot" style="background:#f6b73c"></span>найденный объект</span>
  </div>
</header>
<div id="status">загрузка...</div>
<main><img id="map" alt="карта"></main>
<script>
  const img = document.getElementById('map');
  const status = document.getElementById('status');
  async function update() {
    try {
      const health = await (await fetch('/health', {cache: 'no-store'})).json();
      const parts = [];
      parts.push(health.map_received ? `карта ${health.width}x${health.height}, ${health.resolution_m}м/клетка` : 'карта ещё не построена');
      parts.push(health.robot_pose_available ? 'позиция ровера известна' : 'позиция ровера неизвестна');
      parts.push(`объектов на карте: ${health.marker_count}`);
      status.textContent = parts.join(' · ');
      img.src = '/map.jpg?t=' + Date.now();
    } catch (e) {
      status.textContent = 'офлайн';
    }
  }
  update();
  setInterval(update, 1000);
</script>
</body>
</html>
""".encode("utf-8")


@dataclass
class FoundMarker:
    label: str
    x: float
    y: float
    at: float


@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    grid: OccupancyGrid | None = None
    robot_pose: tuple[float, float, float] | None = None  # x, y, yaw (map frame)
    home_pose: tuple[float, float] | None = None
    markers: list[FoundMarker] = field(default_factory=list)
    latest_jpeg: bytes | None = None


def render_map(state: SharedState) -> bytes | None:
    with state.lock:
        grid = state.grid
        robot_pose = state.robot_pose
        home_pose = state.home_pose
        markers = list(state.markers)

    if grid is None:
        canvas = np.full((240, 320, 3), 20, dtype=np.uint8)
        cv2.putText(canvas, "no map yet", (60, 120), cv2.FONT_HERSHEY_SIMPLEX,
                   0.7, (140, 140, 140), 1, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".jpg", canvas)
        return encoded.tobytes() if ok else None

    width = grid.info.width
    height = grid.info.height
    resolution = grid.info.resolution
    origin_x = grid.info.origin.position.x
    origin_y = grid.info.origin.position.y
    if width <= 0 or height <= 0 or resolution <= 0:
        return None

    data = np.array(grid.data, dtype=np.int16).reshape((height, width))
    # OccupancyGrid: -1 unknown, 0 free, 100 occupied. Map to grayscale where
    # unknown is mid-gray, free is light, occupied is dark.
    image = np.full((height, width), 130, dtype=np.uint8)
    image[data == 0] = 235
    occupied = data >= 50
    image[occupied] = (245 - (data[occupied].clip(50, 100) * 2)).astype(np.uint8)
    # Row 0 of OccupancyGrid.data is the bottom row in world coordinates.
    image = np.flipud(image)
    color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    def to_pixel(x: float, y: float) -> tuple[int, int]:
        px = int((x - origin_x) / resolution)
        py = height - 1 - int((y - origin_y) / resolution)
        return px, py

    for marker in markers:
        px, py = to_pixel(marker.x, marker.y)
        if 0 <= px < width and 0 <= py < height:
            cv2.circle(color, (px, py), 5, (60, 183, 246), -1, cv2.LINE_AA)
            cv2.circle(color, (px, py), 5, (10, 15, 20), 1, cv2.LINE_AA)

    if home_pose is not None:
        px, py = to_pixel(*home_pose)
        if 0 <= px < width and 0 <= py < height:
            cv2.drawMarker(color, (px, py), (255, 199, 124), cv2.MARKER_TRIANGLE_UP,
                           10, 2, cv2.LINE_AA)

    if robot_pose is not None:
        rx, ry, ryaw = robot_pose
        px, py = to_pixel(rx, ry)
        if 0 <= px < width and 0 <= py < height:
            tip = (px + int(12 * math.cos(-ryaw)), py + int(12 * math.sin(-ryaw)))
            cv2.circle(color, (px, py), 6, (150, 217, 56), -1, cv2.LINE_AA)
            cv2.arrowedLine(color, (px, py), tip, (150, 217, 56), 2, cv2.LINE_AA, tipLength=0.5)

    scale = max(1, 720 // max(width, height))
    if scale > 1:
        color = cv2.resize(color, (width * scale, height * scale),
                           interpolation=cv2.INTER_NEAREST)

    ok, encoded = cv2.imencode(".jpg", color, (cv2.IMWRITE_JPEG_QUALITY, 85))
    return encoded.tobytes() if ok else None


class MapViewNode(Node):
    def __init__(self, state: SharedState, map_frame: str, base_frame: str) -> None:
        super().__init__("z_boys_map_view")
        self.state = state
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 5)
        self.create_subscription(String, "/search/found", self._on_found, 20)
        self.create_timer(0.5, self._update_pose)

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self.state.lock:
            self.state.grid = msg

    def _on_found(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            label = str(payload.get("class", "object"))
            x = float(payload["x"])
            y = float(payload["y"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.get_logger().warning(f"ignoring malformed /search/found message: {msg.data!r}")
            return
        with self.state.lock:
            self.state.markers.append(FoundMarker(label, x, y, time.time()))
            now = time.time()
            self.state.markers = [
                m for m in self.state.markers if now - m.at < MARKER_STALE_S
            ][-MAX_MARKERS:]

    def _update_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.2)
            )
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        with self.state.lock:
            self.state.robot_pose = (translation.x, translation.y, yaw)
            if self.state.home_pose is None:
                self.state.home_pose = (translation.x, translation.y)


def render_loop(state: SharedState, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        jpeg = render_map(state)
        if jpeg is not None:
            with state.lock:
                state.latest_jpeg = jpeg
        stop_event.wait(1.0)


class MapHandler(BaseHTTPRequestHandler):
    state: SharedState
    map_frame: str

    def do_GET(self) -> None:  # noqa: N802
        path = urlsplit(self.path).path
        if path == "/":
            self._send(HTTPStatus.OK, "text/html; charset=utf-8", PAGE)
        elif path == "/map.jpg":
            with self.state.lock:
                jpeg = self.state.latest_jpeg
            if jpeg is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "map not ready")
                return
            self._send(HTTPStatus.OK, "image/jpeg", jpeg)
        elif path == "/health":
            self._send_health()
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def _send(self, status: HTTPStatus, content_type: str, payload: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _send_health(self) -> None:
        with self.state.lock:
            grid = self.state.grid
            robot_pose = self.state.robot_pose
            marker_count = len(self.state.markers)
        payload = {
            "map_received": grid is not None,
            "width": grid.info.width if grid else None,
            "height": grid.info.height if grid else None,
            "resolution_m": round(grid.info.resolution, 4) if grid else None,
            "robot_pose_available": robot_pose is not None,
            "robot_pose": (
                {"x": round(robot_pose[0], 3), "y": round(robot_pose[1], 3),
                 "yaw_rad": round(robot_pose[2], 3)}
                if robot_pose else None
            ),
            "marker_count": marker_count,
        }
        body = json.dumps(payload).encode("utf-8")
        self._send(HTTPStatus.OK, "application/json; charset=utf-8", body)

    def log_message(self, format_string: str, *args: Any) -> None:
        LOG.debug("HTTP %s - %s", self.client_address[0], format_string % args)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.getenv("MAP_VIEW_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.getenv("MAP_VIEW_PORT", "8093")))
    parser.add_argument("--map-frame", default=os.getenv("MAP_VIEW_MAP_FRAME", "map"))
    parser.add_argument("--base-frame", default=os.getenv("MAP_VIEW_BASE_FRAME", "base_link"))
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(threadName)s %(message)s")

    state = SharedState()
    stop_event = threading.Event()

    rclpy.init(args=None)
    node = MapViewNode(state, args.map_frame, args.base_frame)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True, name="ros-spin")
    ros_thread.start()

    render_thread = threading.Thread(target=render_loop, args=(state, stop_event),
                                     daemon=True, name="render")
    render_thread.start()

    MapHandler.state = state
    MapHandler.map_frame = args.map_frame
    server = ThreadingHTTPServer((args.host, args.port), MapHandler)
    LOG.info("map view on http://%s:%d", args.host, args.port)
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        stop_event.set()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
