#!/usr/bin/env python3
"""Records the YOLO-annotated MJPEG stream (boxes included) to segmented
video files, separate from the raw camera video telemetry already records.
Uses cv2 (already a dependency of yolo_live_mjpeg.py) instead of the ffmpeg
CLI, which is not installed in this image. Each frame is already a JPEG
(MJPEG source), so this is a decode+re-encode at the same quality, not a
heavy transcode -- cheap on a Pi.
"""

from __future__ import annotations

import argparse
import logging
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2

LOG = logging.getLogger("record-yolo-video")


def open_capture(source: str) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
    return capture


def segment_filename(output_dir: Path) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H-%M-%S-%f")
    return output_dir / f"yolo_cam_{stamp}.avi"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=os.getenv(
        "YOLO_VIDEO_SOURCE", "http://z-boys-yolo-live:8092/stream.mjpg"))
    parser.add_argument("--output-dir", default=os.getenv(
        "YOLO_VIDEO_OUTPUT_DIR", "/logs/session/video"))
    parser.add_argument("--segment-seconds", type=float, default=float(
        os.getenv("YOLO_VIDEO_SEGMENT_S", "120")))
    parser.add_argument("--fps", type=float, default=float(
        os.getenv("YOLO_VIDEO_FPS", "10")))
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    backoff = 0.5
    while True:
        capture = open_capture(args.source)
        if not capture.isOpened():
            capture.release()
            LOG.warning("could not open %s; retrying in %.1fs", args.source, backoff)
            time.sleep(backoff)
            backoff = min(backoff * 2.0, 10.0)
            continue

        ok, frame = capture.read()
        if not ok or frame is None:
            capture.release()
            LOG.warning("no frame from %s; retrying in %.1fs", args.source, backoff)
            time.sleep(backoff)
            backoff = min(backoff * 2.0, 10.0)
            continue

        backoff = 0.5
        height, width = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        writer = None
        segment_started = time.monotonic()

        try:
            while True:
                if writer is None:
                    path = segment_filename(output_dir)
                    writer = cv2.VideoWriter(str(path), fourcc, args.fps, (width, height))
                    LOG.info("recording segment: %s", path)
                    segment_started = time.monotonic()

                writer.write(frame)

                if time.monotonic() - segment_started >= args.segment_seconds:
                    writer.release()
                    writer = None

                ok, frame = capture.read()
                if not ok or frame is None:
                    LOG.warning("stream read failed mid-segment; reconnecting")
                    break
        finally:
            if writer is not None:
                writer.release()
            capture.release()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
