#!/usr/bin/env python3
"""Live map: renders the SLAM occupancy grid as a base image and serves the
other layers as vector JSON (robot pose, driven trail, confirmed objects,
raw camera sightings) so the browser draws/toggles them independently on top
of the same base map. Read-only: this node never commands motion.

Layer honesty, by data source:
  occupancy  - lidar-built SLAM grid. Centimeter accurate.
  trail      - odometry/tf breadcrumb of where the rover has driven. Accurate.
  found      - lidar+camera fused, only published once object_search.py has
               a validated calibration and lidar-associates a target.
               Centimeter accurate, but only exists in approach mode.
  sighted    - camera-only, no calibration needed, but there is no distance
               measurement for a raw YOLO box: this is NOT a point on the
               map, it is a direction from the rover's pose at that moment.
               Drawn as a short ray/cone, never as a dot.
"""

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

MAX_FOUND = 300
MAX_SIGHTED = 300
MAX_TRAIL = 2000
FOUND_STALE_S = 3600.0
SIGHTED_STALE_S = 1800.0
TRAIL_MIN_STEP_M = 0.15
TRAIL_MIN_INTERVAL_S = 1.0

PAGE = """<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RoboMarvel — карта</title>
<style>
  :root { color-scheme: dark; font-family: system-ui, sans-serif; }
  body { margin: 0; background: #0b0f14; color: #e8edf3; }
  header { display: flex; align-items: center; gap: 16px; padding: 10px 16px; border-bottom: 1px solid #1c2530; flex-wrap: wrap; }
  header h1 { font-size: 16px; margin: 0; flex: 1; }
  label { display: inline-flex; align-items: center; gap: 5px; font-size: 12px; color: #aeb8c4; cursor: pointer; user-select: none; }
  .dot { width: 9px; height: 9px; border-radius: 50%; display: inline-block; }
  .ray { width: 12px; height: 2px; display: inline-block; }
  main { display: flex; justify-content: center; padding: 12px; }
  .stage { position: relative; }
  .stage img, .stage canvas { display: block; max-width: 100%; height: auto; border-radius: 6px; }
  .stage canvas { position: absolute; top: 0; left: 0; width: 100%; height: 100%; }
  #status { font-size: 12px; color: #8793a1; padding: 0 16px 10px; }
</style>
</head>
<body>
<header>
  <h1>RoboMarvel &middot; карта</h1>
  <label><input type="checkbox" id="layer-trail" checked><span class="ray" style="background:#7f77dd"></span>след</label>
  <label><input type="checkbox" id="layer-found" checked><span class="dot" style="background:#f6b73c"></span>подтверждённые объекты</label>
  <label><input type="checkbox" id="layer-sighted" checked><span class="ray" style="background:#5dcaa5"></span>замечено камерой (приблизительно)</label>
</header>
<div id="status">загрузка...</div>
<main>
  <div class="stage">
    <img id="base" alt="карта">
    <canvas id="overlay"></canvas>
  </div>
</main>
<script>
  const base = document.getElementById('base');
  const canvas = document.getElementById('overlay');
  const ctx = canvas.getContext('2d');
  const status = document.getElementById('status');
  const cbTrail = document.getElementById('layer-trail');
  const cbFound = document.getElementById('layer-found');
  const cbSighted = document.getElementById('layer-sighted');

  function resizeCanvas() {
    if (canvas.width !== base.clientWidth || canvas.height !== base.clientHeight) {
      canvas.width = base.clientWidth;
      canvas.height = base.clientHeight;
    }
  }
  // Deliberately NOT tied to the <img> 'load' event: reloading base.src every
  // tick fires 'load' asynchronously, and resizing there was racing with (and
  // clearing) the draw call below on almost every frame.
  window.addEventListener('resize', resizeCanvas);

  async function update() {
    try {
      const layers = await (await fetch('/api/layers', {cache: 'no-store'})).json();
      const parts = [];
      parts.push(layers.map_received ? `карта ${layers.width}x${layers.height}, ${layers.resolution_m}м/клетка` : 'карта ещё не построена');
      parts.push(layers.robot_pose ? 'позиция ровера известна' : 'позиция ровера неизвестна');
      parts.push(`след: ${layers.trail.length} точек, объектов: ${layers.found.length}, замечено: ${layers.sighted.length}`);
      status.textContent = parts.join(' · ');
      base.src = '/map.jpg?t=' + Date.now();
      resizeCanvas();

      if (!layers.map_received || canvas.width === 0) return;
      const scaleX = canvas.width / layers.width;
      const scaleY = canvas.height / layers.height;
      const toPx = (x, y) => {
        const px = (x - layers.origin_x) / layers.resolution_m * scaleX;
        const py = canvas.height - (y - layers.origin_y) / layers.resolution_m * scaleY;
        return [px, py];
      };

      ctx.clearRect(0, 0, canvas.width, canvas.height);

      if (cbTrail.checked && layers.trail.length > 1) {
        ctx.strokeStyle = '#7f77dd';
        ctx.lineWidth = 2;
        ctx.beginPath();
        layers.trail.forEach((p, i) => {
          const [px, py] = toPx(p[0], p[1]);
          if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
        });
        ctx.stroke();
      }

      if (cbSighted.checked) {
        ctx.strokeStyle = '#5dcaa5';
        ctx.lineWidth = 2;
        for (const s of layers.sighted) {
          const [px, py] = toPx(s.x, s.y);
          const tip = [px + 22 * Math.cos(-s.yaw), py + 22 * Math.sin(-s.yaw)];
          ctx.beginPath();
          ctx.moveTo(px, py);
          ctx.lineTo(tip[0], tip[1]);
          ctx.stroke();
        }
      }

      if (cbFound.checked) {
        ctx.fillStyle = '#f6b73c';
        for (const f of layers.found) {
          const [px, py] = toPx(f.x, f.y);
          ctx.beginPath();
          ctx.arc(px, py, 5, 0, 2 * Math.PI);
          ctx.fill();
        }
      }

      if (layers.home) {
        const [px, py] = toPx(layers.home.x, layers.home.y);
        ctx.fillStyle = '#7cc7ff';
        ctx.beginPath();
        ctx.moveTo(px, py - 6);
        ctx.lineTo(px - 6, py + 5);
        ctx.lineTo(px + 6, py + 5);
        ctx.closePath();
        ctx.fill();
      }

      if (layers.robot_pose) {
        const [px, py] = toPx(layers.robot_pose.x, layers.robot_pose.y);
        const tip = [px + 14 * Math.cos(-layers.robot_pose.yaw_rad), py + 14 * Math.sin(-layers.robot_pose.yaw_rad)];
        ctx.strokeStyle = '#96d938';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(tip[0], tip[1]);
        ctx.stroke();
        ctx.fillStyle = '#96d938';
        ctx.beginPath();
        ctx.arc(px, py, 6, 0, 2 * Math.PI);
        ctx.fill();
      }
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
class SightedRay:
    label: str
    confidence: float
    x: float
    y: float
    yaw: float
    at: float


@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    grid: OccupancyGrid | None = None
    robot_pose: tuple[float, float, float] | None = None
    home_pose: tuple[float, float] | None = None
    trail: list[tuple[float, float]] = field(default_factory=list)
    found: list[FoundMarker] = field(default_factory=list)
    sighted: list[SightedRay] = field(default_factory=list)
    latest_jpeg: bytes | None = None
    last_trail_at: float = 0.0


def render_base_map(state: SharedState) -> bytes | None:
    """The occupancy-grid layer only -- everything else is drawn client-side
    from /api/layers so each layer can be toggled independently."""
    with state.lock:
        grid = state.grid

    if grid is None:
        canvas = np.full((240, 320, 3), 20, dtype=np.uint8)
        cv2.putText(canvas, "no map yet", (60, 120), cv2.FONT_HERSHEY_SIMPLEX,
                   0.7, (140, 140, 140), 1, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".jpg", canvas)
        return encoded.tobytes() if ok else None

    width, height, resolution = grid.info.width, grid.info.height, grid.info.resolution
    if width <= 0 or height <= 0 or resolution <= 0:
        return None

    data = np.array(grid.data, dtype=np.int16).reshape((height, width))
    image = np.full((height, width), 130, dtype=np.uint8)
    image[data == 0] = 235
    occupied = data >= 50
    image[occupied] = (245 - (data[occupied].clip(50, 100) * 2)).astype(np.uint8)
    # Row 0 of OccupancyGrid.data is the bottom row in world coordinates.
    image = np.flipud(image)
    color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

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
        self.create_subscription(String, "/search/sighted", self._on_sighted, 20)
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
            self.state.found.append(FoundMarker(label, x, y, time.time()))
            now = time.time()
            self.state.found = [
                m for m in self.state.found if now - m.at < FOUND_STALE_S
            ][-MAX_FOUND:]

    def _on_sighted(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            label = str(payload.get("class", "object"))
            confidence = float(payload.get("confidence", 0.0))
            x = float(payload["x"])
            y = float(payload["y"])
            yaw = float(payload["yaw"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.get_logger().warning(f"ignoring malformed /search/sighted message: {msg.data!r}")
            return
        with self.state.lock:
            self.state.sighted.append(SightedRay(label, confidence, x, y, yaw, time.time()))
            now = time.time()
            self.state.sighted = [
                s for s in self.state.sighted if now - s.at < SIGHTED_STALE_S
            ][-MAX_SIGHTED:]

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
        now = time.time()
        with self.state.lock:
            self.state.robot_pose = (translation.x, translation.y, yaw)
            if self.state.home_pose is None:
                self.state.home_pose = (translation.x, translation.y)

            moved_enough = (
                not self.state.trail
                or math.hypot(
                    translation.x - self.state.trail[-1][0],
                    translation.y - self.state.trail[-1][1],
                ) >= TRAIL_MIN_STEP_M
            )
            waited_enough = now - self.state.last_trail_at >= TRAIL_MIN_INTERVAL_S
            if moved_enough and waited_enough:
                self.state.trail.append((translation.x, translation.y))
                self.state.trail = self.state.trail[-MAX_TRAIL:]
                self.state.last_trail_at = now


def render_loop(state: SharedState, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        jpeg = render_base_map(state)
        if jpeg is not None:
            with state.lock:
                state.latest_jpeg = jpeg
        stop_event.wait(1.0)


class MapHandler(BaseHTTPRequestHandler):
    state: SharedState

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
        elif path == "/api/layers":
            self._send_layers()
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def _send(self, status: HTTPStatus, content_type: str, payload: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _send_layers(self) -> None:
        with self.state.lock:
            grid = self.state.grid
            robot_pose = self.state.robot_pose
            home_pose = self.state.home_pose
            trail = list(self.state.trail)
            found = list(self.state.found)
            sighted = list(self.state.sighted)

        payload = {
            "map_received": grid is not None,
            "width": grid.info.width if grid else None,
            "height": grid.info.height if grid else None,
            "resolution_m": round(grid.info.resolution, 4) if grid else None,
            "origin_x": grid.info.origin.position.x if grid else None,
            "origin_y": grid.info.origin.position.y if grid else None,
            "robot_pose": (
                {"x": round(robot_pose[0], 3), "y": round(robot_pose[1], 3),
                 "yaw_rad": round(robot_pose[2], 3)}
                if robot_pose else None
            ),
            "home": (
                {"x": round(home_pose[0], 3), "y": round(home_pose[1], 3)}
                if home_pose else None
            ),
            "trail": [[round(x, 3), round(y, 3)] for x, y in trail],
            "found": [
                {"class": m.label, "x": round(m.x, 3), "y": round(m.y, 3), "at": m.at}
                for m in found
            ],
            "sighted": [
                {"class": s.label, "confidence": round(s.confidence, 2),
                 "x": round(s.x, 3), "y": round(s.y, 3), "yaw": round(s.yaw, 3), "at": s.at}
                for s in sighted
            ],
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
