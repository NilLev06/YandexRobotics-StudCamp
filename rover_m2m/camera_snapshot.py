#!/usr/bin/env python3
"""Shared camera snapshot capture for rover services."""

from __future__ import annotations

import os


DEFAULT_CAMERA_URL = "rtsp://127.0.0.1:8554/cam"
DEFAULT_JPEG_QUALITY = 85
DEFAULT_TIMEOUT_MS = 5000


class CameraSnapshotError(RuntimeError):
    """The current camera frame could not be captured or encoded."""


def _integer_env(name: str, default: int, minimum: int, maximum: int) -> int:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        value = int(raw)
    except ValueError as exc:
        raise CameraSnapshotError(f"{name} must be an integer") from exc
    if not minimum <= value <= maximum:
        raise CameraSnapshotError(f"{name} must be between {minimum} and {maximum}")
    return value


def capture_jpeg(
    camera_url: str | None = None,
    jpeg_quality: int | None = None,
    timeout_ms: int | None = None,
) -> bytes:
    """Read a fresh MediaMTX frame and return it as JPEG bytes."""
    url = camera_url or os.getenv("ROVER_CAMERA_URL", DEFAULT_CAMERA_URL)
    quality = (
        _integer_env("CAMERA_SNAPSHOT_JPEG_QUALITY", DEFAULT_JPEG_QUALITY, 1, 100)
        if jpeg_quality is None
        else jpeg_quality
    )
    timeout = (
        _integer_env("CAMERA_SNAPSHOT_TIMEOUT_MS", DEFAULT_TIMEOUT_MS, 250, 30000)
        if timeout_ms is None
        else timeout_ms
    )
    if not 1 <= quality <= 100:
        raise CameraSnapshotError("jpeg_quality must be between 1 and 100")
    if not 250 <= timeout <= 30000:
        raise CameraSnapshotError("timeout_ms must be between 250 and 30000")

    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise CameraSnapshotError("OpenCV module cv2 is not installed") from exc

    os.environ.setdefault(
        "OPENCV_FFMPEG_CAPTURE_OPTIONS",
        f"rtsp_transport;tcp|stimeout;{timeout * 1000}",
    )
    capture = cv2.VideoCapture()
    try:
        if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
            capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, timeout)
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, timeout)
        if not capture.open(url, cv2.CAP_FFMPEG):
            raise CameraSnapshotError(f"cannot open camera stream: {url}")
        ok, frame = capture.read()
        if not ok or frame is None:
            raise CameraSnapshotError(f"cannot read camera frame: {url}")
        ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        if not ok:
            raise CameraSnapshotError("OpenCV failed to encode JPEG")
        return encoded.tobytes()
    finally:
        capture.release()
