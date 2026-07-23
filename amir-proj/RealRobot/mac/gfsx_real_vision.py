#!/usr/bin/env python3
"""Real GFS-X camera detector.

Reads the Raspberry Pi MJPEG stream, runs the supplied end-to-end YOLO ONNX
model, and sends compact, timestamped observations to Unity over localhost
UDP.  This process never sends ROS or motor/servo commands.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


MODEL_SIZE = 512
BALL_CLASS_ID = 0
SCHEMA_VERSION = 1
EXPECTED_MODEL_SHA256 = "0e8baf30da0f988a54dde2a8ea089112bb1e2cd2ccde5155a5368521961242ca"


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


@dataclass(frozen=True)
class Letterbox:
    scale: float
    pad_x: float
    pad_y: float


def preprocess(frame: np.ndarray) -> tuple[np.ndarray, Letterbox]:
    height, width = frame.shape[:2]
    scale = min(MODEL_SIZE / width, MODEL_SIZE / height)
    resized_width = max(1, int(round(width * scale)))
    resized_height = max(1, int(round(height * scale)))
    resized = cv2.resize(
        frame,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )
    pad_x = (MODEL_SIZE - resized_width) / 2.0
    pad_y = (MODEL_SIZE - resized_height) / 2.0
    left = int(math.floor(pad_x))
    right = MODEL_SIZE - resized_width - left
    top = int(math.floor(pad_y))
    bottom = MODEL_SIZE - resized_height - top
    boxed = cv2.copyMakeBorder(
        resized,
        top,
        bottom,
        left,
        right,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    rgb = cv2.cvtColor(boxed, cv2.COLOR_BGR2RGB)
    tensor = np.ascontiguousarray(
        rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    )
    return tensor, Letterbox(scale=scale, pad_x=float(left), pad_y=float(top))


def unletterbox_box(
    raw_box: np.ndarray,
    mapping: Letterbox,
    frame_width: int,
    frame_height: int,
) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = (float(value) for value in raw_box[:4])
    x1 = clamp((x1 - mapping.pad_x) / mapping.scale, 0.0, frame_width - 1.0)
    x2 = clamp((x2 - mapping.pad_x) / mapping.scale, 0.0, frame_width - 1.0)
    y1 = clamp((y1 - mapping.pad_y) / mapping.scale, 0.0, frame_height - 1.0)
    y2 = clamp((y2 - mapping.pad_y) / mapping.scale, 0.0, frame_height - 1.0)
    return x1, y1, x2, y2


def select_ball(
    output: np.ndarray,
    mapping: Letterbox,
    frame_width: int,
    frame_height: int,
    confidence_threshold: float,
) -> tuple[tuple[float, float, float, float], float] | None:
    rows = np.asarray(output).reshape(-1, 6)
    candidates: list[tuple[float, float, tuple[float, float, float, float]]] = []
    for row in rows:
        confidence = float(row[4])
        class_id = int(round(float(row[5])))
        if class_id != BALL_CLASS_ID or confidence < confidence_threshold:
            continue
        box = unletterbox_box(row[:4], mapping, frame_width, frame_height)
        x1, y1, x2, y2 = box
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        if area > 1.0:
            candidates.append((confidence, area, box))
    if not candidates:
        return None
    confidence, _, box = max(candidates, key=lambda item: (item[0], item[1]))
    return box, confidence


def metric_distance_normalized(
    box_height_px: float,
    frame_width: int,
    frame_height: int,
    horizontal_fov_degrees: float,
    ball_diameter_metres: float,
    maximum_distance_metres: float,
) -> tuple[float, float]:
    if box_height_px <= 0.5:
        return maximum_distance_metres, 1.0
    horizontal_fov = math.radians(horizontal_fov_degrees)
    vertical_fov = 2.0 * math.atan(
        math.tan(horizontal_fov / 2.0) * frame_height / frame_width
    )
    focal_y_px = frame_height / (2.0 * math.tan(vertical_fov / 2.0))
    distance_metres = ball_diameter_metres * focal_y_px / box_height_px
    return (
        distance_metres,
        clamp(distance_metres / maximum_distance_metres, 0.0, 1.0),
    )


class Detector:
    def __init__(self, model_path: Path) -> None:
        self.model_hash = hashlib.sha256(model_path.read_bytes()).hexdigest()
        if self.model_hash != EXPECTED_MODEL_SHA256:
            raise RuntimeError(
                "YOLO model hash mismatch: expected "
                f"{EXPECTED_MODEL_SHA256}, got {self.model_hash}"
            )
        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        options.intra_op_num_threads = max(1, min(4, (os_cpu_count() or 4)))
        self.session = ort.InferenceSession(
            str(model_path),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or list(inputs[0].shape) != [1, 3, 512, 512]:
            raise RuntimeError(f"Unexpected YOLO input: {[item.shape for item in inputs]}")
        if len(outputs) != 1 or list(outputs[0].shape) != [1, 300, 6]:
            raise RuntimeError(f"Unexpected YOLO output: {[item.shape for item in outputs]}")
        self.input_name = inputs[0].name
        self.output_name = outputs[0].name

    def infer(self, frame: np.ndarray) -> tuple[np.ndarray, Letterbox]:
        tensor, mapping = preprocess(frame)
        output = self.session.run([self.output_name], {self.input_name: tensor})[0]
        return output, mapping


def os_cpu_count() -> int | None:
    try:
        import os

        return os.cpu_count()
    except Exception:
        return None


def make_packet(
    sequence: int,
    frame: np.ndarray,
    detection: tuple[tuple[float, float, float, float], float] | None,
    args: argparse.Namespace,
    inference_ms: float,
) -> dict[str, object]:
    height, width = frame.shape[:2]
    packet: dict[str, object] = {
        "schema": SCHEMA_VERSION,
        "seq": sequence,
        "sent_unix_ms": int(time.time() * 1000),
        "frame_width": width,
        "frame_height": height,
        "inference_ms": round(inference_ms, 3),
        "sees": False,
        "angle": 0.0,
        "distance": 1.0,
        "bbox_height_ratio": 0.0,
        "distance_metres": args.maximum_distance_metres,
        "confidence": 0.0,
        "x1": 0.0,
        "y1": 0.0,
        "x2": 0.0,
        "y2": 0.0,
    }
    if detection is None:
        return packet

    (x1, y1, x2, y2), confidence = detection
    center_x = (x1 + x2) * 0.5
    box_height = max(0.0, y2 - y1)
    distance_metres, distance_normalized = metric_distance_normalized(
        box_height,
        width,
        height,
        args.camera_horizontal_fov_degrees,
        args.ball_diameter_metres,
        args.maximum_distance_metres,
    )
    packet.update(
        {
            "sees": True,
            "angle": clamp((center_x - width * 0.5) / (width * 0.5), -1.0, 1.0),
            # The supplied policy's P3 observation is 0 near / 1 far.
            "distance": distance_normalized,
            # Kept separately because the P7 handout uses this opposite
            # convention. Unity logs both values so calibration is auditable.
            "bbox_height_ratio": clamp(box_height / height, 0.0, 1.0),
            "distance_metres": distance_metres,
            "confidence": confidence,
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
        }
    )
    return packet


def annotate(frame: np.ndarray, packet: dict[str, object]) -> np.ndarray:
    rendered = frame.copy()
    if bool(packet["sees"]):
        x1, y1, x2, y2 = (
            int(round(float(packet[key]))) for key in ("x1", "y1", "x2", "y2")
        )
        cv2.rectangle(rendered, (x1, y1), (x2, y2), (30, 220, 30), 2)
        label = (
            f"ball {float(packet['confidence']):.2f}  "
            f"angle {float(packet['angle']):+.2f}  "
            f"dist {float(packet['distance_metres']):.2f}m"
        )
        cv2.putText(rendered, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (30, 220, 30), 1)
    else:
        cv2.putText(rendered, "NO BALL", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 230), 2)
    cv2.putText(
        rendered,
        f"inference {float(packet['inference_ms']):.1f} ms | q = quit",
        (8, rendered.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 255),
        1,
    )
    return rendered


def open_capture(url: str) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(url)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def run(args: argparse.Namespace) -> int:
    model_path = Path(args.model).expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    detector = Detector(model_path)
    udp = None if args.no_udp else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.udp_host, args.udp_port)

    still_image = None
    capture = None
    if args.image:
        still_image = cv2.imread(str(Path(args.image).expanduser()))
        if still_image is None:
            raise RuntimeError(f"Could not read image: {args.image}")
    else:
        capture = open_capture(args.camera_url)

    sequence = 0
    consecutive_detections = 0
    consecutive_failures = 0
    print(
        json.dumps(
            {
                "status": "ready",
                "model": str(model_path),
                "model_sha256": detector.model_hash,
                "camera": args.image or args.camera_url,
                "udp": "disabled" if args.no_udp else f"{args.udp_host}:{args.udp_port}",
                "motor_commands": False,
            }
        ),
        flush=True,
    )

    try:
        while args.max_frames <= 0 or sequence < args.max_frames:
            if still_image is not None:
                frame = still_image.copy()
            else:
                assert capture is not None
                ok, frame = capture.read()
                if not ok or frame is None:
                    consecutive_failures += 1
                    if consecutive_failures >= 5:
                        capture.release()
                        time.sleep(0.25)
                        capture = open_capture(args.camera_url)
                        consecutive_failures = 0
                    continue
                consecutive_failures = 0

            start = time.perf_counter()
            output, mapping = detector.infer(frame)
            inference_ms = (time.perf_counter() - start) * 1000.0
            detection = select_ball(
                output,
                mapping,
                frame.shape[1],
                frame.shape[0],
                args.confidence,
            )
            if detection is None:
                consecutive_detections = 0
            else:
                consecutive_detections += 1
            confirmed_detection = (
                detection
                if consecutive_detections >= args.confirmation_frames
                else None
            )
            packet = make_packet(
                sequence,
                frame,
                confirmed_detection,
                args,
                inference_ms,
            )
            packet["raw_detection"] = detection is not None
            packet["confirmation_count"] = consecutive_detections
            if udp is not None:
                udp.sendto(
                    json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                    target,
                )

            if sequence % args.log_every == 0:
                print(json.dumps(packet, separators=(",", ":")), flush=True)

            if not args.headless:
                cv2.imshow("GFS-X REAL CAMERA / YOLO", annotate(frame, packet))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            sequence += 1
            if still_image is not None and args.max_frames <= 0:
                break
    finally:
        if capture is not None:
            capture.release()
        if udp is not None:
            udp.close()
        cv2.destroyAllWindows()
    return 0


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        default=str(here / "models" / "best_int8_yolo26.onnx"),
    )
    parser.add_argument("--camera-url", default="http://192.168.2.152:8080/")
    parser.add_argument("--image", help="Use one local image instead of MJPEG.")
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument(
        "--no-udp",
        action="store_true",
        help="Inference self-test only; do not send packets to Unity.",
    )
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument(
        "--confirmation-frames",
        type=int,
        default=2,
        help="Require consecutive ball detections before sees=true.",
    )
    parser.add_argument("--camera-horizontal-fov-degrees", type=float, default=62.0)
    parser.add_argument("--ball-diameter-metres", type=float, default=0.065)
    parser.add_argument("--maximum-distance-metres", type=float, default=2.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--log-every", type=int, default=15)
    args = parser.parse_args()
    if not (0.0 < args.confidence <= 1.0):
        parser.error("--confidence must be in (0, 1]")
    if not (10.0 <= args.camera_horizontal_fov_degrees < 180.0):
        parser.error("--camera-horizontal-fov-degrees must be in [10, 180)")
    if args.ball_diameter_metres <= 0.0 or args.maximum_distance_metres <= 0.0:
        parser.error("Ball diameter and maximum distance must be positive.")
    if args.log_every < 1:
        parser.error("--log-every must be positive")
    if not (1 <= args.confirmation_frames <= 10):
        parser.error("--confirmation-frames must be in [1, 10]")
    return args


if __name__ == "__main__":
    try:
        raise SystemExit(run(parse_args()))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"fatal: {exc}", file=sys.stderr)
        raise
