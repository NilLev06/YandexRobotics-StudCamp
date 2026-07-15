#!/usr/bin/env python3
"""Lightweight live dashboard for the rover: one process, no external DB.

Subscribes to a handful of ROS topics plus the YOLO /health endpoint and Pi
system metrics, keeps a bounded in-memory history, and serves a single HTML
page with Chart.js (loaded from a CDN) that polls a small JSON API. No
Prometheus/Grafana/timeseries-DB needed — this is meant to be cheap and
disposable, not a permanent observability stack.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import subprocess
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlsplit

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist

LOG = logging.getLogger("dashboard")

HISTORY_LEN = 300  # ~5 minutes at 1 sample/sec

PAGE = """<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RoboMarvel — панель ровера</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  :root { color-scheme: dark; font-family: system-ui, sans-serif; }
  body { margin: 0; background: #0b0f14; color: #e8edf3; }
  main { max-width: 1100px; margin: auto; padding: 12px 16px 40px; }
  h1 { font-size: 20px; margin: 16px 0; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 14px; }
  .card { background: #121822; border-radius: 10px; padding: 12px 14px; }
  .card h2 { font-size: 13px; text-transform: uppercase; letter-spacing: .04em; color: #8793a1; margin: 0 0 8px; }
  .stat { font-size: 26px; font-weight: 600; }
  .stat.warn { color: #f6b73c; }
  .stat.bad { color: #ff6767; }
  .stat.ok { color: #38d996; }
  .sub { color: #8793a1; font-size: 12px; margin-top: 2px; }
  canvas { max-height: 160px; }
  .badge { display: inline-block; padding: 2px 8px; border-radius: 999px; font-size: 12px; margin: 2px 4px 2px 0; }
  .badge.ok { background: #123324; color: #38d996; }
  .badge.bad { background: #3a1414; color: #ff6767; }
  #status { color: #8793a1; font-size: 12px; }
</style>
</head>
<body>
<main>
  <h1>RoboMarvel &middot; панель ровера <span id="status"></span></h1>
  <div class="grid">
    <div class="card"><h2>Батарея, В</h2><div class="stat" id="battery-stat">—</div><canvas id="battery-chart"></canvas></div>
    <div class="card"><h2>Температура CPU, &deg;C</h2><div class="stat" id="temp-stat">—</div><canvas id="temp-chart"></canvas></div>
    <div class="card"><h2>Клиренс лидара, м</h2><div class="stat" id="lidar-stat">—</div><canvas id="lidar-chart"></canvas></div>
    <div class="card"><h2>Свободно на диске, ГБ</h2><div class="stat" id="disk-stat">—</div><canvas id="disk-chart"></canvas></div>
    <div class="card"><h2>Команда движения</h2><div class="stat" id="cmdvel-stat">—</div><div class="sub" id="cmdvel-sub"></div></div>
    <div class="card"><h2>Wi-Fi, дБм</h2><div class="stat" id="wifi-stat">—</div><canvas id="wifi-chart"></canvas></div>
    <div class="card"><h2>Видит YOLO</h2><div id="yolo-badges">—</div><div class="sub" id="yolo-sub"></div></div>
    <div class="card"><h2>Статус цели</h2><div class="stat" id="goal-stat" style="font-size:16px">—</div></div>
  </div>
</main>
<script>
const charts = {};
function makeChart(id, label, color) {
  const ctx = document.getElementById(id).getContext('2d');
  charts[id] = new Chart(ctx, {
    type: 'line',
    data: { labels: [], datasets: [{ label, data: [], borderColor: color, borderWidth: 1.5, pointRadius: 0, tension: 0.25 }] },
    options: {
      animation: false, responsive: true,
      scales: { x: { display: false }, y: { ticks: { color: '#8793a1', font: { size: 10 } }, grid: { color: '#1c2530' } } },
      plugins: { legend: { display: false } },
    },
  });
}
makeChart('battery-chart', 'В', '#38d996');
makeChart('temp-chart', '°C', '#f6b73c');
makeChart('lidar-chart', 'м', '#7cc7ff');
makeChart('disk-chart', 'ГБ', '#c792ea');
makeChart('wifi-chart', 'дБм', '#ff9f68');

function pushSeries(id, points) {
  const chart = charts[id];
  chart.data.labels = points.map(p => '');
  chart.data.datasets[0].data = points.map(p => p[1]);
  chart.update('none');
}

function setStat(id, value, cls) {
  const el = document.getElementById(id);
  el.textContent = value;
  el.className = 'stat' + (cls ? ' ' + cls : '');
}

async function update() {
  try {
    const res = await fetch('/api/state', { cache: 'no-store' });
    const s = await res.json();
    document.getElementById('status').textContent = 'обновлено ' + new Date().toLocaleTimeString();

    setStat('battery-stat', s.battery_v == null ? '—' : s.battery_v.toFixed(2),
      s.battery_v == null ? '' : (s.battery_v < 6.8 ? 'bad' : (s.battery_v < 7.2 ? 'warn' : 'ok')));
    pushSeries('battery-chart', s.history.battery_v);

    setStat('temp-stat', s.cpu_temp_c == null ? '—' : s.cpu_temp_c.toFixed(1),
      s.cpu_temp_c == null ? '' : (s.cpu_temp_c > 75 ? 'bad' : (s.cpu_temp_c > 65 ? 'warn' : 'ok')));
    pushSeries('temp-chart', s.history.cpu_temp_c);

    setStat('lidar-stat', s.lidar_min_m == null ? '—' : s.lidar_min_m.toFixed(2),
      s.lidar_min_m == null ? '' : (s.lidar_min_m < 0.20 ? 'bad' : (s.lidar_min_m < 0.35 ? 'warn' : 'ok')));
    pushSeries('lidar-chart', s.history.lidar_min_m);

    setStat('disk-stat', s.disk_free_gb == null ? '—' : s.disk_free_gb.toFixed(1),
      s.disk_free_gb == null ? '' : (s.disk_free_gb < 2 ? 'bad' : (s.disk_free_gb < 5 ? 'warn' : 'ok')));
    pushSeries('disk-chart', s.history.disk_free_gb);

    setStat('wifi-stat', s.wifi_dbm == null ? '—' : s.wifi_dbm.toFixed(0));
    pushSeries('wifi-chart', s.history.wifi_dbm);

    const v = s.cmd_vel;
    setStat('cmdvel-stat', v ? `v=${v.linear.toFixed(2)} м/с  ω=${v.angular.toFixed(2)} рад/с` : 'стоит');
    document.getElementById('cmdvel-sub').textContent = v ? new Date(v.at * 1000).toLocaleTimeString() : '';

    const badges = document.getElementById('yolo-badges');
    badges.innerHTML = '';
    if (s.yolo && s.yolo.detections && s.yolo.detections.length) {
      for (const d of s.yolo.detections) {
        const span = document.createElement('span');
        span.className = 'badge ok';
        span.textContent = `${d.class} ${(d.confidence * 100).toFixed(0)}%`;
        badges.appendChild(span);
      }
    } else {
      badges.innerHTML = '<span class="badge">ничего</span>';
    }
    document.getElementById('yolo-sub').textContent = s.yolo
      ? `${s.yolo.status}, ${s.yolo.inference_fps?.toFixed(1) ?? '—'} fps` : 'нет данных';

    document.getElementById('goal-stat').textContent = s.goal_status || '—';
  } catch (e) {
    document.getElementById('status').textContent = 'офлайн';
  }
}
update();
setInterval(update, 1000);
</script>
</body>
</html>
""".encode("utf-8")


@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    battery_v: float | None = None
    lidar_min_m: float | None = None
    cmd_vel: dict | None = None
    goal_status: str = ""
    yolo: dict | None = None
    cpu_temp_c: float | None = None
    disk_free_gb: float | None = None
    wifi_dbm: float | None = None
    history: dict[str, deque] = field(default_factory=lambda: {
        name: deque(maxlen=HISTORY_LEN)
        for name in ("battery_v", "cpu_temp_c", "lidar_min_m", "disk_free_gb", "wifi_dbm")
    })


def record(state: SharedState, name: str, value: float | None) -> None:
    if value is None:
        return
    with state.lock:
        state.history[name].append((time.time(), value))


class DashboardNode(Node):
    def __init__(self, state: SharedState) -> None:
        super().__init__("z_boys_dashboard")
        self.state = state
        self.create_subscription(Float32, "/hardware/battery", self._on_battery,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(String, "/goal/status", self._on_goal_status, 10)

    def _on_battery(self, msg: Float32) -> None:
        with self.state.lock:
            self.state.battery_v = float(msg.data)
        record(self.state, "battery_v", float(msg.data))

    def _on_scan(self, msg: LaserScan) -> None:
        finite = [r for r in msg.ranges if math.isfinite(r) and r >= max(0.0, msg.range_min)]
        minimum = min(finite) if finite else None
        with self.state.lock:
            self.state.lidar_min_m = minimum
        record(self.state, "lidar_min_m", minimum)

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self.state.lock:
            self.state.cmd_vel = {
                "linear": float(msg.linear.x),
                "angular": float(msg.angular.z),
                "at": time.time(),
            }

    def _on_goal_status(self, msg: String) -> None:
        with self.state.lock:
            self.state.goal_status = msg.data


def poll_yolo(state: SharedState, stop_event: threading.Event, url: str) -> None:
    while not stop_event.is_set():
        try:
            request = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
            with urllib.request.urlopen(request, timeout=1.5) as response:
                payload = json.loads(response.read(256 * 1024))
            with state.lock:
                state.yolo = payload
        except (OSError, urllib.error.URLError, json.JSONDecodeError):
            pass
        stop_event.wait(1.0)


def read_cpu_temp_c() -> float | None:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="ascii") as handle:
            return round(int(handle.readline().strip()) / 1000.0, 1)
    except (OSError, ValueError):
        return None


def read_disk_free_gb(path: str) -> float | None:
    try:
        usage = os.statvfs(path)
        return round(usage.f_bavail * usage.f_frsize / (1024 ** 3), 2)
    except OSError:
        return None


def read_wifi_dbm() -> float | None:
    try:
        with open("/proc/net/wireless", "r", encoding="ascii") as handle:
            lines = handle.readlines()
    except OSError:
        return None
    for line in lines[2:]:
        parts = line.split()
        if len(parts) >= 4:
            try:
                return float(parts[3].rstrip("."))
            except ValueError:
                return None
    return None


def poll_system(state: SharedState, stop_event: threading.Event, disk_path: str) -> None:
    while not stop_event.is_set():
        temp = read_cpu_temp_c()
        disk = read_disk_free_gb(disk_path)
        wifi = read_wifi_dbm()
        with state.lock:
            state.cpu_temp_c = temp
            state.disk_free_gb = disk
            state.wifi_dbm = wifi
        record(state, "cpu_temp_c", temp)
        record(state, "disk_free_gb", disk)
        record(state, "wifi_dbm", wifi)
        stop_event.wait(1.0)


class DashboardHandler(BaseHTTPRequestHandler):
    state: SharedState

    def do_GET(self) -> None:  # noqa: N802
        path = urlsplit(self.path).path
        if path == "/":
            self._send(HTTPStatus.OK, "text/html; charset=utf-8", PAGE)
        elif path == "/api/state":
            self._send_state()
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def _send(self, status: HTTPStatus, content_type: str, payload: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _send_state(self) -> None:
        with self.state.lock:
            payload = {
                "battery_v": self.state.battery_v,
                "cpu_temp_c": self.state.cpu_temp_c,
                "lidar_min_m": self.state.lidar_min_m,
                "disk_free_gb": self.state.disk_free_gb,
                "wifi_dbm": self.state.wifi_dbm,
                "cmd_vel": self.state.cmd_vel,
                "goal_status": self.state.goal_status,
                "yolo": self.state.yolo,
                "history": {
                    name: list(points) for name, points in self.state.history.items()
                },
            }
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self._send(HTTPStatus.OK, "application/json; charset=utf-8", body)

    def log_message(self, format_string: str, *args: Any) -> None:
        LOG.debug("HTTP %s - %s", self.client_address[0], format_string % args)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.getenv("DASHBOARD_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.getenv("DASHBOARD_PORT", "8092")))
    parser.add_argument("--yolo-url", default=os.getenv("DASHBOARD_YOLO_URL",
                                                          "http://z-boys-yolo-live:8091/health"))
    parser.add_argument("--disk-path", default=os.getenv("DASHBOARD_DISK_PATH", "/"))
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(threadName)s %(message)s")

    state = SharedState()
    stop_event = threading.Event()

    rclpy.init(args=None)
    node = DashboardNode(state)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True,
                                  name="ros-spin")
    ros_thread.start()

    yolo_thread = threading.Thread(target=poll_yolo, args=(state, stop_event, args.yolo_url),
                                   daemon=True, name="yolo-poll")
    system_thread = threading.Thread(target=poll_system, args=(state, stop_event, args.disk_path),
                                     daemon=True, name="system-poll")
    yolo_thread.start()
    system_thread.start()

    DashboardHandler.state = state
    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    LOG.info("dashboard on http://%s:%d", args.host, args.port)
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
