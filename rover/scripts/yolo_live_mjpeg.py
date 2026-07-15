#!/usr/bin/env python3
"""Serve a low-latency YOLO overlay for the rover camera as MJPEG."""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import threading
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlsplit

# These must be set before importing OpenCV/Ultralytics.
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "rtsp_transport;tcp|timeout;5000000",
)
os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp")

import cv2  # noqa: E402
from ultralytics import YOLO  # noqa: E402


LOG = logging.getLogger("yolo-live")

PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>RoboMarvel YOLO Live</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    body { margin: 0; background: #0b0f14; color: #e8edf3; }
    main { width: min(1280px, 100%); margin: auto; }
    header { display: flex; gap: 12px; align-items: center; padding: 14px 16px; }
    h1 { font-size: 18px; margin: 0; flex: 1; }
    #dot { width: 10px; height: 10px; border-radius: 50%; background: #f6b73c; }
    #dot.ok { background: #38d996; box-shadow: 0 0 10px #38d99699; }
    #status { color: #aeb8c4; font: 13px ui-monospace, monospace; }
    figure { margin: 0; background: #05070a; line-height: 0; }
    img { display: block; width: 100%; height: auto; min-height: 180px; object-fit: contain; }
    footer { padding: 10px 16px 18px; color: #8793a1; font-size: 12px; }
    a { color: #7cc7ff; }
  </style>
</head>
<body>
  <main>
    <header><span id="dot"></span><h1>RoboMarvel · YOLO11n</h1><span id="status">connecting…</span></header>
    <figure><img src="/stream.mjpg" alt="YOLO annotated rover camera"></figure>
    <footer>LAN-only stream · <a href="/snapshot.jpg">snapshot</a> · <a href="/health">health JSON</a></footer>
  </main>
  <script>
    const dot = document.querySelector('#dot');
    const status = document.querySelector('#status');
    async function update() {
      try {
        const response = await fetch('/health', {cache: 'no-store'});
        const data = await response.json();
        dot.className = data.status === 'ok' ? 'ok' : '';
        const age = data.detection_age_ms == null ? '—' : Math.round(data.detection_age_ms) + 'ms age';
        status.textContent = `${data.status} · ${data.detection_count} objects · ${data.inference_fps.toFixed(1)} YOLO fps · ${age}`;
      } catch (_) {
        dot.className = '';
        status.textContent = 'offline';
      }
    }
    update(); setInterval(update, 1000);
  </script>
</body>
</html>
""".encode("utf-8")


@dataclass(frozen=True)
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    class_id: int
    label: str
    confidence: float


@dataclass
class SharedState:
    condition: threading.Condition = field(default_factory=threading.Condition)
    started_at: float = field(default_factory=time.monotonic)

    latest_frame: Any | None = None
    capture_seq: int = 0
    capture_at: float = 0.0
    capture_wall_time_ns: int = 0
    capture_fps: float = 0.0
    capture_connected: bool = False
    capture_error: str = "waiting for RTSP source"

    model_ready: bool = False
    inference_error: str = "model is loading"
    inference_ms: float = 0.0
    inference_fps: float = 0.0
    detections: list[Detection] = field(default_factory=list)
    detection_source_seq: int = 0
    detection_at: float = 0.0
    detection_source_at: float = 0.0
    detection_source_wall_time_ns: int = 0
    detection_completed_wall_time_ns: int = 0
    detection_shape: tuple[int, int] = (0, 0)

    latest_jpeg: bytes | None = None
    output_seq: int = 0
    output_at: float = 0.0
    output_capture_wall_time_ns: int = 0
    output_fps: float = 0.0
    render_error: str = "waiting for first frame"
    active_clients: int = 0

    ros_ready: bool = False
    ros_topic: str = ""
    ros_error: str = "ROS publisher is starting"
    ros_publish_at: float = 0.0
    ros_publish_fps: float = 0.0
    ros_publish_count: int = 0


def ema(previous: float, current: float, weight: float = 0.15) -> float:
    return current if previous <= 0 else previous * (1.0 - weight) + current * weight


def capture_loop(
    state: SharedState,
    stop_event: threading.Event,
    source: str,
) -> None:
    backoff = 0.5

    while not stop_event.is_set():
        capture = cv2.VideoCapture()
        try:
            open_parameters: list[int] = []
            if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
                open_parameters.extend((cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000))
            if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
                open_parameters.extend((cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000))

            try:
                opened = capture.open(
                    source,
                    cv2.CAP_FFMPEG,
                    open_parameters,
                )
            except (TypeError, cv2.error):
                # Compatibility fallback for OpenCV builds without the params overload.
                opened = capture.open(source, cv2.CAP_FFMPEG)

            if not opened:
                raise RuntimeError("could not open RTSP source")

            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            LOG.info("RTSP source connected: %s", source)
            backoff = 0.5

            while not stop_event.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    raise RuntimeError("RTSP frame read failed")

                now = time.monotonic()
                wall_time_ns = time.time_ns()
                with state.condition:
                    if state.capture_at > 0 and now > state.capture_at:
                        state.capture_fps = ema(
                            state.capture_fps,
                            1.0 / (now - state.capture_at),
                        )
                    state.latest_frame = frame
                    state.capture_seq += 1
                    state.capture_at = now
                    state.capture_wall_time_ns = wall_time_ns
                    state.capture_connected = True
                    state.capture_error = ""
                    state.condition.notify_all()

        except Exception as exc:  # OpenCV errors vary by backend.
            message = str(exc)
            LOG.warning("RTSP disconnected: %s; retrying in %.1fs", message, backoff)
            with state.condition:
                state.capture_connected = False
                state.capture_error = message
                state.condition.notify_all()
        finally:
            capture.release()

        if stop_event.wait(backoff):
            break
        backoff = min(backoff * 2.0, 5.0)


def inference_loop(
    state: SharedState,
    stop_event: threading.Event,
    model_path: str,
    confidence: float,
    image_size: int,
    max_fps: float,
) -> None:
    model: YOLO | None = None
    retry_delay = 1.0

    while model is None and not stop_event.is_set():
        try:
            model = YOLO(model_path, task="detect")
            with state.condition:
                state.model_ready = True
                state.inference_error = "waiting for first camera frame"
                state.condition.notify_all()
            LOG.info("YOLO model loaded: %s", model_path)
        except Exception as exc:
            message = str(exc)
            LOG.exception("YOLO model load failed; retrying in %.1fs", retry_delay)
            with state.condition:
                state.model_ready = False
                state.inference_error = message
                state.condition.notify_all()
            if stop_event.wait(retry_delay):
                return
            retry_delay = min(retry_delay * 2.0, 10.0)

    if model is None:
        return

    minimum_interval = 1.0 / max_fps
    last_source_seq = -1

    while not stop_event.is_set():
        with state.condition:
            state.condition.wait_for(
                lambda: stop_event.is_set()
                or (
                    state.latest_frame is not None
                    and state.capture_seq != last_source_seq
                ),
                timeout=0.5,
            )
            if stop_event.is_set():
                break
            if state.latest_frame is None or state.capture_seq == last_source_seq:
                continue

            frame = state.latest_frame.copy()
            source_seq = state.capture_seq
            source_at = state.capture_at
            source_wall_time_ns = state.capture_wall_time_ns

        started = time.monotonic()
        try:
            result = model(
                frame,
                conf=confidence,
                imgsz=image_size,
                verbose=False,
            )[0]
            completed = time.monotonic()
            inference_ms = (completed - started) * 1000.0
            detections: list[Detection] = []

            for box in result.boxes:
                class_id = int(box.cls.item())
                x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
                detections.append(
                    Detection(
                        x1=x1,
                        y1=y1,
                        x2=x2,
                        y2=y2,
                        class_id=class_id,
                        label=str(result.names[class_id]),
                        confidence=float(box.conf.item()),
                    )
                )

            with state.condition:
                if state.detection_at > 0 and completed > state.detection_at:
                    state.inference_fps = ema(
                        state.inference_fps,
                        1.0 / (completed - state.detection_at),
                    )
                else:
                    state.inference_fps = 1000.0 / max(inference_ms, 0.001)
                state.inference_ms = inference_ms
                state.detections = detections
                state.detection_source_seq = source_seq
                state.detection_at = completed
                state.detection_source_at = source_at
                state.detection_source_wall_time_ns = source_wall_time_ns
                state.detection_completed_wall_time_ns = time.time_ns()
                state.detection_shape = frame.shape[:2]
                state.inference_error = ""
                state.condition.notify_all()

            last_source_seq = source_seq
        except Exception as exc:
            LOG.exception("YOLO inference failed")
            with state.condition:
                state.inference_error = str(exc)
                state.condition.notify_all()
            last_source_seq = source_seq

        elapsed = time.monotonic() - started
        stop_event.wait(max(0.0, minimum_interval - elapsed))


PALETTE = (
    (86, 211, 255),
    (101, 226, 139),
    (255, 153, 102),
    (204, 153, 255),
    (92, 224, 224),
    (255, 119, 168),
)


def draw_overlay(
    frame: Any,
    detections: list[Detection],
    detection_shape: tuple[int, int],
    detection_age: float | None,
    inference_ms: float,
    inference_fps: float,
    capture_connected: bool,
    boxes_fresh: bool,
) -> Any:
    height, width = frame.shape[:2]
    source_height, source_width = detection_shape
    scale_x = width / source_width if source_width else 1.0
    scale_y = height / source_height if source_height else 1.0

    scaled: list[tuple[Detection, tuple[int, int, int, int], tuple[int, int, int]]] = []
    if boxes_fresh:
        for detection in detections:
            x1 = max(0, min(width - 1, round(detection.x1 * scale_x)))
            y1 = max(0, min(height - 1, round(detection.y1 * scale_y)))
            x2 = max(0, min(width - 1, round(detection.x2 * scale_x)))
            y2 = max(0, min(height - 1, round(detection.y2 * scale_y)))
            color = PALETTE[detection.class_id % len(PALETTE)]
            scaled.append((detection, (x1, y1, x2, y2), color))

    if scaled:
        tint = frame.copy()
        for _, (x1, y1, x2, y2), color in scaled:
            cv2.rectangle(tint, (x1, y1), (x2, y2), color, -1)
        frame = cv2.addWeighted(tint, 0.10, frame, 0.90, 0.0)

    for detection, (x1, y1, x2, y2), color in scaled:
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        label = f"{detection.label} {detection.confidence:.2f}"
        (text_width, text_height), baseline = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            1,
        )
        label_top = max(0, y1 - text_height - baseline - 8)
        cv2.rectangle(
            frame,
            (x1, label_top),
            (min(width - 1, x1 + text_width + 10), y1),
            color,
            -1,
        )
        cv2.putText(
            frame,
            label,
            (x1 + 5, max(text_height + 2, y1 - baseline - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (10, 15, 20),
            1,
            cv2.LINE_AA,
        )

    status = "LIVE" if capture_connected else "SOURCE OFFLINE"
    age_ms = "--" if detection_age is None else f"{detection_age * 1000:.0f}ms"
    object_count = len(scaled)
    detection_status = "fresh" if boxes_fresh else "stale"
    line_one = f"YOLO11n  {inference_fps:.1f} fps  {inference_ms:.0f} ms"
    line_two = f"{object_count} objects  boxes {detection_status}  age {age_ms}"

    hud = frame.copy()
    cv2.rectangle(hud, (12, 12), (min(width - 12, 480), 90), (5, 10, 16), -1)
    frame = cv2.addWeighted(hud, 0.66, frame, 0.34, 0.0)
    status_color = (80, 230, 145) if capture_connected else (75, 155, 255)
    cv2.circle(frame, (30, 33), 7, status_color, -1, cv2.LINE_AA)
    cv2.putText(
        frame,
        status,
        (45, 39),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (235, 242, 248),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        line_one,
        (24, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (215, 224, 232),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        line_two,
        (24, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (180, 194, 208),
        1,
        cv2.LINE_AA,
    )
    return frame


def compositor_loop(
    state: SharedState,
    stop_event: threading.Event,
    stream_fps: float,
    jpeg_quality: int,
    max_detection_age: float,
) -> None:
    interval = 1.0 / stream_fps
    deadline = time.monotonic()

    while not stop_event.is_set():
        with state.condition:
            if state.latest_frame is None:
                state.condition.wait(timeout=0.25)
                continue
            frame = state.latest_frame.copy()
            detections = list(state.detections)
            detection_shape = state.detection_shape
            detection_source_at = state.detection_source_at
            inference_ms = state.inference_ms
            inference_fps = state.inference_fps
            capture_connected = state.capture_connected
            capture_wall_time_ns = state.capture_wall_time_ns

        now = time.monotonic()
        detection_age = (
            now - detection_source_at if detection_source_at > 0 else None
        )
        boxes_fresh = (
            detection_age is not None and detection_age <= max_detection_age
        )
        try:
            composed = draw_overlay(
                frame,
                detections,
                detection_shape,
                detection_age,
                inference_ms,
                inference_fps,
                capture_connected,
                boxes_fresh,
            )
            ok, encoded = cv2.imencode(
                ".jpg",
                composed,
                (cv2.IMWRITE_JPEG_QUALITY, jpeg_quality),
            )
        except Exception as exc:
            LOG.exception("frame composition failed")
            with state.condition:
                state.render_error = str(exc)
                state.condition.notify_all()
            deadline = time.monotonic()
            stop_event.wait(min(0.5, interval))
            continue

        completed = time.monotonic()
        with state.condition:
            if ok:
                if state.output_at > 0 and completed > state.output_at:
                    state.output_fps = ema(
                        state.output_fps,
                        1.0 / (completed - state.output_at),
                    )
                state.latest_jpeg = encoded.tobytes()
                state.output_seq += 1
                state.output_at = completed
                state.output_capture_wall_time_ns = capture_wall_time_ns
                state.render_error = ""
            else:
                state.render_error = "JPEG encoding failed"
            state.condition.notify_all()

        deadline += interval
        delay = deadline - time.monotonic()
        if delay < -interval:
            deadline = time.monotonic()
            delay = 0.0
        stop_event.wait(max(0.0, delay))


def ros_publish_loop(
    state: SharedState,
    stop_event: threading.Event,
    topic: str,
    frame_id: str,
    max_fps: float,
) -> None:
    """Publish the already-encoded overlay without duplicating inference."""
    retry_delay = 1.0
    with state.condition:
        state.ros_topic = topic
        state.condition.notify_all()

    while not stop_event.is_set():
        context = None
        node = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.signals import SignalHandlerOptions
            from sensor_msgs.msg import CompressedImage

            context = Context()
            rclpy.init(
                args=None,
                context=context,
                signal_handler_options=SignalHandlerOptions.NO,
            )
            node = rclpy.create_node("z_boys_yolo_live", context=context)
            publisher = node.create_publisher(
                CompressedImage,
                topic,
                qos_profile_sensor_data,
            )
            with state.condition:
                state.ros_ready = True
                state.ros_error = ""
                state.condition.notify_all()
            LOG.info("ROS image publisher ready: %s", topic)
            retry_delay = 1.0

            interval = 1.0 / max_fps
            next_publish_at = time.monotonic()
            last_output_seq = -1

            while not stop_event.is_set() and context.ok():
                delay = next_publish_at - time.monotonic()
                if delay > 0 and stop_event.wait(delay):
                    break

                with state.condition:
                    state.condition.wait_for(
                        lambda: stop_event.is_set()
                        or (
                            state.latest_jpeg is not None
                            and state.output_seq != last_output_seq
                        ),
                        timeout=0.5,
                    )
                    if stop_event.is_set():
                        break
                    if (
                        state.latest_jpeg is None
                        or state.output_seq == last_output_seq
                    ):
                        continue
                    jpeg = state.latest_jpeg
                    output_seq = state.output_seq
                    capture_wall_time_ns = state.output_capture_wall_time_ns

                message = CompressedImage()
                if capture_wall_time_ns > 0:
                    message.header.stamp.sec = capture_wall_time_ns // 1_000_000_000
                    message.header.stamp.nanosec = (
                        capture_wall_time_ns % 1_000_000_000
                    )
                else:
                    message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id = frame_id
                message.format = "jpeg"
                message.data = jpeg
                publisher.publish(message)

                published_at = time.monotonic()
                with state.condition:
                    if state.ros_publish_at > 0 and published_at > state.ros_publish_at:
                        state.ros_publish_fps = ema(
                            state.ros_publish_fps,
                            1.0 / (published_at - state.ros_publish_at),
                        )
                    else:
                        state.ros_publish_fps = max_fps
                    state.ros_publish_at = published_at
                    state.ros_publish_count += 1
                    state.ros_error = ""
                    state.condition.notify_all()

                last_output_seq = output_seq
                next_publish_at = published_at + interval

        except Exception as exc:
            LOG.exception("ROS image publisher failed; retrying in %.1fs", retry_delay)
            with state.condition:
                state.ros_ready = False
                state.ros_error = str(exc)
                state.condition.notify_all()
        finally:
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                context.shutdown()
            with state.condition:
                state.ros_ready = False
                state.condition.notify_all()

        if stop_event.wait(retry_delay):
            break
        retry_delay = min(retry_delay * 2.0, 10.0)


def age_ms(timestamp: float, now: float) -> float | None:
    return round(max(0.0, now - timestamp) * 1000.0, 1) if timestamp > 0 else None


class OverlayHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 16

    def get_request(self) -> tuple[Any, Any]:
        request, client_address = super().get_request()
        request.settimeout(10.0)
        return request, client_address


class OverlayHandler(BaseHTTPRequestHandler):
    state: SharedState
    stop_event: threading.Event
    health_stale_after: float
    max_detection_age: float
    max_stream_clients: int

    def do_GET(self) -> None:  # noqa: N802
        path = urlsplit(self.path).path
        if path == "/":
            self.send_payload(HTTPStatus.OK, "text/html; charset=utf-8", PAGE)
        elif path == "/stream.mjpg":
            self.serve_stream()
        elif path == "/snapshot.jpg":
            self.serve_snapshot()
        elif path == "/health":
            self.serve_health()
        elif path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def send_payload(
        self,
        status: HTTPStatus,
        content_type: str,
        payload: bytes,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        self.wfile.write(payload)

    def serve_snapshot(self) -> None:
        with self.state.condition:
            jpeg = self.state.latest_jpeg
        if jpeg is None:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "no frame available")
            return
        self.send_payload(HTTPStatus.OK, "image/jpeg", jpeg)

    def serve_health(self) -> None:
        now = time.monotonic()
        with self.state.condition:
            capture_age = age_ms(self.state.capture_at, now)
            detection_age = age_ms(self.state.detection_source_at, now)
            output_age = age_ms(self.state.output_at, now)
            ros_publish_age = age_ms(self.state.ros_publish_at, now)
            detections_fresh = (
                detection_age is not None
                and detection_age <= self.max_detection_age * 1000.0
            )
            current_detections = (
                self.state.detections if detections_fresh else []
            )
            image_height, image_width = self.state.detection_shape
            detections = [
                {
                    "class": detection.label,
                    "class_id": detection.class_id,
                    "confidence": round(detection.confidence, 3),
                    "bbox_xyxy": [
                        round(detection.x1, 1),
                        round(detection.y1, 1),
                        round(detection.x2, 1),
                        round(detection.y2, 1),
                    ],
                    "center_xy": [
                        round((detection.x1 + detection.x2) / 2.0, 1),
                        round((detection.y1 + detection.y2) / 2.0, 1),
                    ],
                }
                for detection in current_detections
            ]
            inference_ready = self.state.detection_at > 0
            healthy = (
                self.state.model_ready
                and inference_ready
                and self.state.capture_connected
                and detections_fresh
                and output_age is not None
                and output_age <= self.health_stale_after * 1000.0
                and detection_age is not None
                and detection_age <= self.health_stale_after * 1000.0
                and self.state.ros_ready
                and ros_publish_age is not None
                and ros_publish_age <= self.health_stale_after * 1000.0
            )
            payload = {
                "status": "ok" if healthy else "degraded",
                "model_ready": self.state.model_ready,
                "inference_ready": inference_ready,
                "capture_connected": self.state.capture_connected,
                "capture_fps": round(self.state.capture_fps, 1),
                "capture_age_ms": capture_age,
                "inference_ms": round(self.state.inference_ms, 1),
                "inference_fps": round(self.state.inference_fps, 2),
                "detections_fresh": detections_fresh,
                "detection_count": len(current_detections),
                "detection_age_ms": detection_age,
                "detection_seq": self.state.detection_source_seq,
                "detection_source_wall_time_ns": (
                    self.state.detection_source_wall_time_ns or None
                ),
                "detection_completed_wall_time_ns": (
                    self.state.detection_completed_wall_time_ns or None
                ),
                "image_width": image_width,
                "image_height": image_height,
                "detections": detections,
                "output_fps": round(self.state.output_fps, 1),
                "output_age_ms": output_age,
                "active_clients": self.state.active_clients,
                "uptime_s": round(now - self.state.started_at, 1),
                "ros": {
                    "ready": self.state.ros_ready,
                    "topic": self.state.ros_topic,
                    "publish_fps": round(self.state.ros_publish_fps, 2),
                    "publish_age_ms": ros_publish_age,
                    "published_frames": self.state.ros_publish_count,
                },
                "errors": {
                    "capture": self.state.capture_error or None,
                    "inference": self.state.inference_error or None,
                    "render": self.state.render_error or None,
                    "ros": self.state.ros_error or None,
                },
            }

        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        status = HTTPStatus.OK if healthy else HTTPStatus.SERVICE_UNAVAILABLE
        self.send_payload(status, "application/json; charset=utf-8", body)

    def serve_stream(self) -> None:
        with self.state.condition:
            if self.state.active_clients >= self.max_stream_clients:
                accepted = False
            else:
                accepted = True
                self.state.active_clients += 1
                self.state.condition.notify_all()

        if not accepted:
            self.send_error(
                HTTPStatus.SERVICE_UNAVAILABLE,
                "MJPEG viewer limit reached",
            )
            return

        last_seq = -1
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()
            self.connection.settimeout(15.0)

            while not self.stop_event.is_set():
                with self.state.condition:
                    self.state.condition.wait_for(
                        lambda: self.stop_event.is_set()
                        or (
                            self.state.latest_jpeg is not None
                            and self.state.output_seq != last_seq
                        ),
                        timeout=1.0,
                    )
                    if self.stop_event.is_set():
                        break
                    if (
                        self.state.latest_jpeg is None
                        or self.state.output_seq == last_seq
                    ):
                        continue
                    jpeg = self.state.latest_jpeg
                    last_seq = self.state.output_seq

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            pass
        finally:
            with self.state.condition:
                self.state.active_clients = max(0, self.state.active_clients - 1)
                self.state.condition.notify_all()

    def log_message(self, format_string: str, *args: object) -> None:
        LOG.debug("HTTP %s - %s", self.client_address[0], format_string % args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        default=os.getenv("RTSP_URL", "rtsp://ros:8554/cam"),
    )
    parser.add_argument(
        "--model",
        default=os.getenv(
            "YOLO_MODEL",
            "/root/weights/yolo11n_ncnn_model",
        ),
    )
    parser.add_argument("--host", default=os.getenv("HTTP_HOST", "0.0.0.0"))
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.getenv("HTTP_PORT", "8091")),
    )
    parser.add_argument(
        "--confidence",
        type=float,
        default=float(os.getenv("YOLO_CONFIDENCE", "0.5")),
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=int(os.getenv("YOLO_IMAGE_SIZE", "640")),
    )
    parser.add_argument(
        "--inference-fps",
        type=float,
        default=float(os.getenv("YOLO_INFERENCE_FPS", "2")),
    )
    parser.add_argument(
        "--stream-fps",
        type=float,
        default=float(os.getenv("MJPEG_STREAM_FPS", "10")),
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=int(os.getenv("MJPEG_QUALITY", "75")),
    )
    parser.add_argument(
        "--max-detection-age",
        type=float,
        default=float(os.getenv("MAX_DETECTION_AGE", "1.5")),
    )
    parser.add_argument(
        "--health-stale-after",
        type=float,
        default=float(os.getenv("HEALTH_STALE_AFTER", "5")),
    )
    parser.add_argument(
        "--max-stream-clients",
        type=int,
        default=int(os.getenv("MAX_STREAM_CLIENTS", "4")),
    )
    parser.add_argument(
        "--ros-topic",
        default=os.getenv(
            "ROS_IMAGE_TOPIC",
            "/camera/yolo/image_annotated/compressed",
        ),
    )
    parser.add_argument(
        "--ros-frame-id",
        default=os.getenv("ROS_IMAGE_FRAME_ID", "camera_frame"),
    )
    parser.add_argument(
        "--ros-publish-fps",
        type=float,
        default=float(os.getenv("ROS_IMAGE_FPS", "5")),
    )
    parser.add_argument("--log-level", default=os.getenv("LOG_LEVEL", "INFO"))
    args = parser.parse_args()

    if not 0.0 <= args.confidence <= 1.0:
        parser.error("--confidence must be between 0 and 1")
    if args.inference_fps <= 0 or args.stream_fps <= 0:
        parser.error("frame rates must be greater than zero")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be between 1 and 100")
    if args.max_detection_age <= 0 or args.health_stale_after <= 0:
        parser.error("age thresholds must be greater than zero")
    if args.max_stream_clients <= 0:
        parser.error("--max-stream-clients must be greater than zero")
    if not args.ros_topic.startswith("/"):
        parser.error("--ros-topic must be an absolute ROS topic")
    if args.ros_publish_fps <= 0:
        parser.error("--ros-publish-fps must be greater than zero")
    return args


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(threadName)s %(message)s",
    )

    state = SharedState()
    stop_event = threading.Event()
    OverlayHandler.state = state
    OverlayHandler.stop_event = stop_event
    OverlayHandler.health_stale_after = args.health_stale_after
    OverlayHandler.max_detection_age = args.max_detection_age
    OverlayHandler.max_stream_clients = args.max_stream_clients

    server = OverlayHTTPServer((args.host, args.port), OverlayHandler)
    workers = [
        threading.Thread(
            target=capture_loop,
            name="capture",
            args=(state, stop_event, args.source),
            daemon=True,
        ),
        threading.Thread(
            target=inference_loop,
            name="inference",
            args=(
                state,
                stop_event,
                args.model,
                args.confidence,
                args.imgsz,
                args.inference_fps,
            ),
            daemon=True,
        ),
        threading.Thread(
            target=compositor_loop,
            name="compositor",
            args=(
                state,
                stop_event,
                args.stream_fps,
                args.jpeg_quality,
                args.max_detection_age,
            ),
            daemon=True,
        ),
        threading.Thread(
            target=ros_publish_loop,
            name="ros-publisher",
            args=(
                state,
                stop_event,
                args.ros_topic,
                args.ros_frame_id,
                args.ros_publish_fps,
            ),
            daemon=True,
        ),
    ]

    def request_shutdown(signum: int, _frame: object) -> None:
        LOG.info("received signal %s; stopping", signum)
        stop_event.set()
        with state.condition:
            state.condition.notify_all()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)

    for worker in workers:
        worker.start()

    LOG.info(
        "serving http://%s:%d (source=%s, inference=%.1ffps, stream=%.1ffps)",
        args.host,
        args.port,
        args.source,
        args.inference_fps,
        args.stream_fps,
    )
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        stop_event.set()
        with state.condition:
            state.condition.notify_all()
        server.server_close()
        for worker in workers:
            worker.join(timeout=6.0)
        LOG.info("stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
