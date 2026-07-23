#!/usr/bin/env python3
"""Read-only USB-camera MJPEG server for the GFS-X vision process.

This program deliberately has no GPIO, servo, ROS, or motor imports. Its only
job is to expose camera frames at http://ROBOT_IP:8080/ for the Mac detector.
"""

import argparse
import json
import signal
import threading
import time
from http import server
from socketserver import ThreadingMixIn

import cv2


BOUNDARY = b"gfsx-frame"


class FrameSource:
    def __init__(self, device, width, height, fps, quality):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self.condition = threading.Condition()
        self.frame = None
        self.sequence = 0
        self.last_frame_time = 0.0
        self.error = None
        self.stopping = False
        self.capture = None
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stopping = True
        capture = self.capture
        if capture is not None:
            capture.release()
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=2.0)

    def healthy(self):
        return (
            self.error is None
            and self.last_frame_time > 0.0
            and time.monotonic() - self.last_frame_time < 2.0
        )

    def wait_for_frame(self, after_sequence, timeout=2.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while (
                not self.stopping
                and self.sequence <= after_sequence
                and time.monotonic() < deadline
            ):
                self.condition.wait(max(0.0, deadline - time.monotonic()))
            return self.sequence, self.frame

    def _capture_loop(self):
        capture = cv2.VideoCapture(self.device)
        self.capture = capture
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        if not capture.isOpened():
            self.error = "cannot open camera device %s" % self.device
            with self.condition:
                self.condition.notify_all()
            return

        encode_options = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        failures = 0
        while not self.stopping:
            ok, image = capture.read()
            if not ok:
                failures += 1
                if failures >= 30:
                    self.error = "camera stopped returning frames"
                    break
                time.sleep(0.03)
                continue
            failures = 0
            encoded, jpeg = cv2.imencode(".jpg", image, encode_options)
            if not encoded:
                continue
            with self.condition:
                self.frame = jpeg.tobytes()
                self.sequence += 1
                self.last_frame_time = time.monotonic()
                self.condition.notify_all()

        capture.release()
        with self.condition:
            self.condition.notify_all()


class CameraHandler(server.BaseHTTPRequestHandler):
    server_version = "GFSXCamera/1.0"

    def log_message(self, format_string, *args):
        print(
            "%s - %s" % (self.address_string(), format_string % args),
            flush=True,
        )

    def do_GET(self):
        if self.path == "/healthz":
            healthy = self.server.source.healthy()
            payload = json.dumps(
                {
                    "ok": healthy,
                    "frames": self.server.source.sequence,
                    "error": self.server.source.error,
                }
            ).encode("utf-8")
            self.send_response(200 if healthy else 503)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        if self.path not in ("/", "/stream.mjpg"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=%s"
            % BOUNDARY.decode("ascii"),
        )
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

        sequence = -1
        try:
            while not self.server.source.stopping:
                sequence, frame = self.server.source.wait_for_frame(sequence)
                if frame is None:
                    if self.server.source.error is not None:
                        break
                    continue
                self.wfile.write(b"--" + BOUNDARY + b"\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    ("Content-Length: %d\r\n\r\n" % len(frame)).encode("ascii")
                )
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass


class ThreadedHttpServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler, source):
        self.source = source
        super().__init__(address, handler)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--quality", type=int, default=80)
    return parser.parse_args()


def main():
    args = parse_args()
    source = FrameSource(
        args.device,
        max(1, args.width),
        max(1, args.height),
        max(1, args.fps),
        max(20, min(95, args.quality)),
    )
    httpd = ThreadedHttpServer(
        (args.host, args.port),
        CameraHandler,
        source,
    )

    def stop(_signum, _frame):
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    source.start()
    try:
        httpd.serve_forever(poll_interval=0.25)
    finally:
        source.stop()
        httpd.server_close()


if __name__ == "__main__":
    main()
