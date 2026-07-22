#!/usr/bin/env python3
"""Persistent RTSP local detector used as a cloud-vision gate."""

from __future__ import annotations

import os
import sys
import threading
import time
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2  # type: ignore
import numpy as np  # type: ignore

VENDOR_PYTHON = "/src/vendor-python"
if Path(VENDOR_PYTHON).is_dir():
    sys.path.insert(0, VENDOR_PYTHON)

try:
    import onnxruntime as ort  # type: ignore
except ImportError as exc:  # pragma: no cover - only exercised on the robot
    raise RuntimeError("onnxruntime is required for the local visual gate") from exc


CLASS_NAMES = ("ball", "cube", "robot-claw")
TARGET_CLASSES = {
    "мяч": "ball",
    "мячик": "ball",
    "куб": "cube",
    "кубик": "cube",
    "ball": "ball",
    "cube": "cube",
}


@dataclass
class Detection:
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]


@dataclass
class Candidate:
    timestamp: float
    detection: Detection
    jpeg: bytes
    frame_width: int
    frame_height: int


class YoloDetector:
    def __init__(
        self,
        model_path: str,
        confidence: float = 0.22,
        iou_threshold: float = 0.45,
        threads: int = 3,
    ) -> None:
        options = ort.SessionOptions()
        options.intra_op_num_threads = max(1, threads)
        options.inter_op_num_threads = 1
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(
            model_path, sess_options=options, providers=["CPUExecutionProvider"]
        )
        model_input = self.session.get_inputs()[0]
        self.input_name = model_input.name
        self.height = int(model_input.shape[2])
        self.width = int(model_input.shape[3])
        self.output_shape = self.session.get_outputs()[0].shape
        self.end_to_end = len(self.output_shape) == 3 and self.output_shape[-1] == 6
        self.confidence = confidence
        self.iou_threshold = iou_threshold

    def preprocess(self, frame: np.ndarray) -> tuple[np.ndarray, float, int, int]:
        source_height, source_width = frame.shape[:2]
        scale = min(self.width / source_width, self.height / source_height)
        resized_width = max(1, round(source_width * scale))
        resized_height = max(1, round(source_height * scale))
        resized = cv2.resize(frame, (resized_width, resized_height))
        pad_x = (self.width - resized_width) // 2
        pad_y = (self.height - resized_height) // 2
        canvas = np.full((self.height, self.width, 3), 114, dtype=np.uint8)
        canvas[pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = resized
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32)
        tensor /= 255.0
        return tensor, scale, pad_x, pad_y

    @staticmethod
    def restore_box(
        box: tuple[float, float, float, float],
        scale: float,
        pad_x: int,
        pad_y: int,
        frame_width: int,
        frame_height: int,
    ) -> tuple[int, int, int, int]:
        x1, y1, x2, y2 = box
        x1 = int(round((x1 - pad_x) / scale))
        y1 = int(round((y1 - pad_y) / scale))
        x2 = int(round((x2 - pad_x) / scale))
        y2 = int(round((y2 - pad_y) / scale))
        return (
            max(0, min(frame_width - 1, x1)),
            max(0, min(frame_height - 1, y1)),
            max(1, min(frame_width, x2)),
            max(1, min(frame_height, y2)),
        )

    def detect(self, frame: np.ndarray) -> list[Detection]:
        tensor, scale, pad_x, pad_y = self.preprocess(frame)
        output = self.session.run(None, {self.input_name: tensor})[0]
        frame_height, frame_width = frame.shape[:2]
        if self.end_to_end:
            rows = np.asarray(output[0])
            detections = []
            for x1, y1, x2, y2, score, class_id in rows:
                class_index = int(class_id)
                if float(score) < self.confidence or not 0 <= class_index < len(CLASS_NAMES):
                    continue
                bbox = self.restore_box(
                    (float(x1), float(y1), float(x2), float(y2)),
                    scale,
                    pad_x,
                    pad_y,
                    frame_width,
                    frame_height,
                )
                if bbox[2] > bbox[0] and bbox[3] > bbox[1]:
                    detections.append(Detection(CLASS_NAMES[class_index], float(score), bbox))
            return detections

        rows = np.asarray(output[0]).T
        boxes: list[list[int]] = []
        scores: list[float] = []
        classes: list[int] = []
        restored: list[tuple[int, int, int, int]] = []
        for row in rows:
            class_index = int(np.argmax(row[4:]))
            score = float(row[4 + class_index])
            if score < self.confidence:
                continue
            cx, cy, width, height = (float(value) for value in row[:4])
            bbox = self.restore_box(
                (cx - width / 2, cy - height / 2, cx + width / 2, cy + height / 2),
                scale,
                pad_x,
                pad_y,
                frame_width,
                frame_height,
            )
            restored.append(bbox)
            boxes.append([bbox[0], bbox[1], bbox[2] - bbox[0], bbox[3] - bbox[1]])
            scores.append(score)
            classes.append(class_index)
        keep: list[int] = []
        for class_index in set(classes):
            indices = [index for index, value in enumerate(classes) if value == class_index]
            selected = cv2.dnn.NMSBoxes(
                [boxes[index] for index in indices],
                [scores[index] for index in indices],
                self.confidence,
                self.iou_threshold,
            )
            keep.extend(indices[int(index)] for index in np.asarray(selected).reshape(-1))
        return [
            Detection(CLASS_NAMES[classes[index]], scores[index], restored[index])
            for index in keep
        ]


def crop_candidate(frame: np.ndarray, detection: Detection, padding: float = 0.30) -> bytes:
    frame_height, frame_width = frame.shape[:2]
    x1, y1, x2, y2 = detection.bbox
    width, height = x2 - x1, y2 - y1
    x1 = max(0, int(x1 - width * padding))
    y1 = max(0, int(y1 - height * padding))
    x2 = min(frame_width, int(x2 + width * padding))
    y2 = min(frame_height, int(y2 + height * padding))
    crop = frame[y1:y2, x1:x2]
    ok, encoded = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 88])
    if not ok:
        raise RuntimeError("failed to encode local detector crop")
    return encoded.tobytes()


class LocalVisualGate:
    def __init__(
        self,
        model_path: str,
        camera_url: str,
        confidence: float = 0.22,
        inference_fps: float = 2.0,
        confirmation_hits: int = 2,
    ) -> None:
        self.detector = YoloDetector(model_path, confidence=confidence)
        self.camera_url = camera_url
        self.interval = 1.0 / max(0.2, inference_fps)
        self.confirmation_hits = max(1, confirmation_hits)
        self.events: dict[str, deque[Candidate]] = defaultdict(lambda: deque(maxlen=20))
        self.frame_events: deque[tuple[float, bytes]] = deque(maxlen=40)
        self.condition = threading.Condition()
        self.stopping = False
        self.last_error: str | None = None
        self.frames = 0
        self.inferences = 0
        self.inference_seconds = 0.0
        self.thread = threading.Thread(target=self._run, name="local-visual-gate", daemon=True)
        self.thread.start()

    def _run(self) -> None:
        os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp")
        while not self.stopping:
            capture = cv2.VideoCapture(self.camera_url, cv2.CAP_FFMPEG)
            if not capture.isOpened():
                self.last_error = f"cannot open {self.camera_url}"
                time.sleep(1.0)
                continue
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            next_inference = 0.0
            try:
                while not self.stopping:
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        self.last_error = "camera frame read failed"
                        break
                    self.frames += 1
                    now = time.monotonic()
                    if now < next_inference:
                        continue
                    started = time.monotonic()
                    try:
                        detections = self.detector.detect(frame)
                    except Exception as exc:  # keep the stream worker recoverable
                        self.last_error = f"local inference failed: {exc}"
                        time.sleep(0.2)
                        continue
                    self.inferences += 1
                    self.inference_seconds += time.monotonic() - started
                    next_inference = now + self.interval
                    frame_ok, frame_jpeg = cv2.imencode(
                        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82]
                    )
                    best: dict[str, Detection] = {}
                    for detection in detections:
                        current = best.get(detection.class_name)
                        if current is None or detection.confidence > current.confidence:
                            best[detection.class_name] = detection
                    with self.condition:
                        if frame_ok:
                            self.frame_events.append(
                                (time.monotonic(), frame_jpeg.tobytes())
                            )
                        for class_name, detection in best.items():
                            self.events[class_name].append(
                                Candidate(
                                    timestamp=time.monotonic(),
                                    detection=detection,
                                    jpeg=crop_candidate(frame, detection),
                                    frame_width=frame.shape[1],
                                    frame_height=frame.shape[0],
                                )
                            )
                        self.condition.notify_all()
            finally:
                capture.release()
            if not self.stopping:
                time.sleep(0.5)

    def wait_candidate(
        self, target: str, after: float, timeout: float
    ) -> Candidate | None:
        class_name = TARGET_CLASSES.get(target.casefold())
        if class_name is None:
            return None
        deadline = time.monotonic() + timeout
        with self.condition:
            while not self.stopping:
                recent = [event for event in self.events[class_name] if event.timestamp >= after]
                if len(recent) >= self.confirmation_hits:
                    return recent[-1]
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.condition.wait(timeout=min(remaining, 0.25))
        return None

    def frames_since(self, after: float, max_frames: int = 4) -> list[bytes]:
        """Return evenly spaced full frames captured since a motion started."""
        with self.condition:
            frames = [jpeg for timestamp, jpeg in self.frame_events if timestamp >= after]
        if len(frames) <= max_frames:
            return frames
        indices = np.linspace(0, len(frames) - 1, max_frames, dtype=int)
        return [frames[int(index)] for index in indices]

    def latest_candidate(
        self, target: str, after: float, required_hits: int | None = None
    ) -> Candidate | None:
        """Return a confirmed candidate without blocking the navigation loop."""
        class_name = TARGET_CLASSES.get(target.casefold())
        if class_name is None:
            return None
        with self.condition:
            recent = [event for event in self.events[class_name] if event.timestamp >= after]
            hits = self.confirmation_hits if required_hits is None else max(1, required_hits)
            if len(recent) >= hits:
                return recent[-1]
        return None

    def stats(self) -> dict[str, Any]:
        average_ms = (
            self.inference_seconds / self.inferences * 1000.0 if self.inferences else None
        )
        return {
            "frames": self.frames,
            "inferences": self.inferences,
            "average_inference_ms": round(average_ms, 1) if average_ms else None,
            "last_error": self.last_error,
        }

    def close(self) -> None:
        self.stopping = True
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=3.0)
