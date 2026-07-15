#!/usr/bin/env python3
"""Fail-closed object search and optional lidar-assisted approach for RoboMarvel.

The target class (any YOLO/COCO label, e.g. "bottle" or "chair") is passed via
--target-class. The node deliberately uses Nav2 actions instead of commanding
rotational velocity. Approach mode is locked until an explicit, validated
camera/lidar calibration is mounted. Exit status 2 is reserved for one very
specific outcome: every step of a complete 360 degree search succeeded and the
target object was never confirmed.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import statistics
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Callable, Optional, Sequence

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener


EXIT_OK = 0
EXIT_ERROR = 1
EXIT_NOT_FOUND = 2

ACTIVE_GOAL_STATES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


class UnsafeError(RuntimeError):
    """Raised when continuing could be unsafe or ambiguous."""


class SafeArgumentParser(argparse.ArgumentParser):
    """Keep argparse's conventional code 2 exclusive to a real no-find result."""

    def error(self, message: str) -> None:
        self.print_usage(sys.stderr)
        self.exit(64, f"{self.prog}: error: {message}\n")


@dataclass(frozen=True)
class BottleObservation:
    seq: int
    confidence: float
    bbox: tuple[float, float, float, float]
    width: int
    height: int


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, float, float, float, float]


@dataclass(frozen=True)
class RigidTransform:
    translation: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


@dataclass(frozen=True)
class AssociationLimits:
    min_lidar_points: int
    min_scans: int
    max_index_gap: int
    bbox_padding_fraction: float
    max_range_mad_m: float
    max_range_spread_m: float
    max_scan_target_drift_m: float
    max_target_bearing_rad: float


@dataclass(frozen=True)
class Calibration:
    intrinsics: Intrinsics
    lidar_frame: str
    camera_frame: str
    lidar_to_camera: RigidTransform
    association: AssociationLimits
    swept_radius_m: float


@dataclass(frozen=True)
class LidarTarget:
    x_base: float
    y_base: float
    range_m: float
    bearing_rad: float
    points: int


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise UnsafeError(f"calibration {name} must be numeric")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise UnsafeError(f"calibration {name} must be numeric") from exc
    if not math.isfinite(number):
        raise UnsafeError(f"calibration {name} must be finite")
    return number


def _quaternion_norm(q: Sequence[float]) -> float:
    return math.sqrt(sum(component * component for component in q))


def _rotate_vector(
    quaternion: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    """Rotate a vector by an xyzw unit quaternion."""
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    # Expanded q * v * conjugate(q), avoiding a dependency on numpy/scipy.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _apply_transform(
    transform: RigidTransform, point: Sequence[float]
) -> tuple[float, float, float]:
    rx, ry, rz = _rotate_vector(transform.quaternion, point)
    tx, ty, tz = transform.translation
    return rx + tx, ry + ty, rz + tz


def _ros_transform(transform: Any) -> RigidTransform:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    q = (rotation.x, rotation.y, rotation.z, rotation.w)
    norm = _quaternion_norm(q)
    if not math.isfinite(norm) or norm < 0.99 or norm > 1.01:
        raise UnsafeError("live TF contains an invalid quaternion")
    return RigidTransform(
        (translation.x, translation.y, translation.z),
        tuple(component / norm for component in q),
    )


def _yaw_from_quaternion(q: Sequence[float]) -> float:
    qx, qy, qz, qw = q
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _duration_message(seconds: float) -> Any:
    return Duration(seconds=seconds).to_msg()


def _bbox_similar(a: BottleObservation, b: BottleObservation) -> bool:
    if a.width != b.width or a.height != b.height:
        return False
    ax1, ay1, ax2, ay2 = a.bbox
    bx1, by1, bx2, by2 = b.bbox
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - intersection
    iou = intersection / union if union > 0.0 else 0.0
    acx, acy = (ax1 + ax2) * 0.5, (ay1 + ay2) * 0.5
    bcx, bcy = (bx1 + bx2) * 0.5, (by1 + by2) * 0.5
    normalized_center_distance = math.hypot(acx - bcx, acy - bcy) / math.hypot(
        a.width, a.height
    )
    return iou >= 0.25 or normalized_center_distance <= 0.08


def _average_observations(items: Sequence[BottleObservation]) -> BottleObservation:
    return BottleObservation(
        seq=items[-1].seq,
        confidence=min(item.confidence for item in items),
        bbox=tuple(
            statistics.median(item.bbox[index] for item in items)
            for index in range(4)
        ),
        width=items[-1].width,
        height=items[-1].height,
    )


def load_calibration(path: str, image_width: int, image_height: int) -> Calibration:
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise UnsafeError(f"cannot read calibration {path}: {exc}") from exc

    if payload.get("schema_version") != 1:
        raise UnsafeError("calibration schema_version must be 1")
    if payload.get("calibrated") is not True:
        raise UnsafeError(
            "approach is locked: calibration must explicitly set calibrated=true"
        )

    validation = payload.get("validation") or {}
    if validation.get("validated") is not True:
        raise UnsafeError(
            "approach is locked: calibration validation.validated must be true"
        )
    sample_count = int(validation.get("sample_count") or 0)
    rmse = _finite_number(
        validation.get("rms_reprojection_error_px"),
        "validation.rms_reprojection_error_px",
    )
    if sample_count < 20 or rmse <= 0.0 or rmse > 3.0:
        raise UnsafeError(
            "calibration validation requires >=20 samples and 0 < RMSE <= 3 px"
        )
    if not str(validation.get("validated_at_utc") or "").strip():
        raise UnsafeError("calibration validation timestamp is missing")

    safety = payload.get("safety_validation") or {}
    required_true = (
        "lidar_bearing_validated",
        "scan_plane_intersects_floor_target",
    )
    missing_safety = [name for name in required_true if safety.get(name) is not True]
    if missing_safety:
        raise UnsafeError(
            "approach is locked: safety validation is incomplete ("
            + ", ".join(missing_safety)
            + ")"
        )
    swept_radius_m = _finite_number(
        safety.get("measured_chassis_swept_radius_m"),
        "safety_validation.measured_chassis_swept_radius_m",
    )
    supervised_trials = int(safety.get("supervised_standoff_trials") or 0)
    if not (0.08 <= swept_radius_m <= 0.50):
        raise UnsafeError("measured chassis swept radius is outside safe bounds")
    if supervised_trials < 3:
        raise UnsafeError("approach requires at least 3 supervised standoff trials")

    camera = payload.get("camera") or {}
    intrinsics_payload = camera.get("intrinsics") or {}
    width = int(camera.get("width_px") or 0)
    height = int(camera.get("height_px") or 0)
    if width != image_width or height != image_height:
        raise UnsafeError(
            f"calibration resolution {width}x{height} does not match YOLO "
            f"{image_width}x{image_height}"
        )
    fx = _finite_number(intrinsics_payload.get("fx_px"), "camera.intrinsics.fx_px")
    fy = _finite_number(intrinsics_payload.get("fy_px"), "camera.intrinsics.fy_px")
    cx = _finite_number(intrinsics_payload.get("cx_px"), "camera.intrinsics.cx_px")
    cy = _finite_number(intrinsics_payload.get("cy_px"), "camera.intrinsics.cy_px")
    if not (100.0 <= fx <= 10000.0 and 100.0 <= fy <= 10000.0):
        raise UnsafeError("calibration focal lengths are implausible")
    if not (0.0 <= cx < width and 0.0 <= cy < height):
        raise UnsafeError("calibration principal point lies outside the image")
    if camera.get("distortion_model") != "plumb_bob":
        raise UnsafeError("only a calibrated plumb_bob camera model is supported")
    coefficients = camera.get("distortion_coefficients")
    if not isinstance(coefficients, list) or len(coefficients) != 5:
        raise UnsafeError("calibration requires five plumb_bob coefficients")
    distortion = tuple(
        _finite_number(value, f"camera.distortion_coefficients[{index}]")
        for index, value in enumerate(coefficients)
    )

    extrinsics = payload.get("extrinsics") or {}
    lidar_frame = str(extrinsics.get("lidar_frame") or "").strip().lstrip("/")
    camera_frame = str(extrinsics.get("camera_optical_frame") or "").strip().lstrip(
        "/"
    )
    transform = extrinsics.get("lidar_to_camera") or {}
    translation_payload = transform.get("translation_m")
    quaternion_payload = transform.get("quaternion_xyzw")
    if not lidar_frame or not camera_frame:
        raise UnsafeError("calibration lidar and camera frame names are required")
    if not isinstance(translation_payload, list) or len(translation_payload) != 3:
        raise UnsafeError("calibration lidar_to_camera translation must have 3 values")
    if not isinstance(quaternion_payload, list) or len(quaternion_payload) != 4:
        raise UnsafeError("calibration lidar_to_camera quaternion must have 4 values")
    translation = tuple(
        _finite_number(value, f"extrinsics.translation_m[{index}]")
        for index, value in enumerate(translation_payload)
    )
    quaternion = tuple(
        _finite_number(value, f"extrinsics.quaternion_xyzw[{index}]")
        for index, value in enumerate(quaternion_payload)
    )
    norm = _quaternion_norm(quaternion)
    if abs(norm - 1.0) > 0.01:
        raise UnsafeError("calibration quaternion must be normalized within 1%")
    if math.sqrt(sum(value * value for value in translation)) > 2.0:
        raise UnsafeError("calibration camera/lidar translation is implausibly large")
    quaternion = tuple(value / norm for value in quaternion)

    association_payload = payload.get("association") or {}
    limits = AssociationLimits(
        min_lidar_points=int(association_payload.get("min_lidar_points", 3)),
        min_scans=int(association_payload.get("min_scans", 3)),
        max_index_gap=int(association_payload.get("max_index_gap", 2)),
        bbox_padding_fraction=_finite_number(
            association_payload.get("bbox_padding_fraction", 0.08),
            "association.bbox_padding_fraction",
        ),
        max_range_mad_m=_finite_number(
            association_payload.get("max_range_mad_m", 0.08),
            "association.max_range_mad_m",
        ),
        max_range_spread_m=_finite_number(
            association_payload.get("max_range_spread_m", 0.25),
            "association.max_range_spread_m",
        ),
        max_scan_target_drift_m=_finite_number(
            association_payload.get("max_scan_target_drift_m", 0.12),
            "association.max_scan_target_drift_m",
        ),
        max_target_bearing_rad=math.radians(
            _finite_number(
                association_payload.get("max_target_bearing_deg", 35.0),
                "association.max_target_bearing_deg",
            )
        ),
    )
    if not (3 <= limits.min_lidar_points <= 30 and 2 <= limits.min_scans <= 10):
        raise UnsafeError("calibration lidar association counts are outside safe bounds")
    if not (1 <= limits.max_index_gap <= 3):
        raise UnsafeError("calibration max_index_gap must be between 1 and 3")
    if not (0.0 <= limits.bbox_padding_fraction <= 0.20):
        raise UnsafeError("calibration bbox padding must be between 0 and 0.20")
    if not (0.01 <= limits.max_range_mad_m <= 0.15):
        raise UnsafeError("calibration range MAD bound is outside safe limits")
    if not (0.03 <= limits.max_range_spread_m <= 0.40):
        raise UnsafeError("calibration range spread bound is outside safe limits")
    if not (0.03 <= limits.max_scan_target_drift_m <= 0.20):
        raise UnsafeError("calibration target drift bound is outside safe limits")
    if not math.radians(5.0) <= limits.max_target_bearing_rad <= math.radians(60.0):
        raise UnsafeError("calibration target bearing bound is outside safe limits")

    return Calibration(
        Intrinsics(width, height, fx, fy, cx, cy, distortion),
        lidar_frame,
        camera_frame,
        RigidTransform(translation, quaternion),
        limits,
        swept_radius_m,
    )


class BottleSearchNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("z_boys_object_search")
        self.args = args
        self.stop_requested = threading.Event()
        self._lock = threading.Lock()
        self._scan: Optional[LaserScan] = None
        self._scan_at = 0.0
        self._odom: Optional[Odometry] = None
        self._odom_at = 0.0
        self._battery: Optional[Float32] = None
        self._battery_at = 0.0
        self._spin_status: Optional[GoalStatusArray] = None
        self._navigate_status: Optional[GoalStatusArray] = None
        self._active_goal: Any = None
        self._active_result_future: Any = None
        self._base_from_scan: Optional[RigidTransform] = None
        self._expected_scan_frame: Optional[str] = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.spin_client = ActionClient(self, Spin, "/spin")
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.spin_cancel_client = self.create_client(
            CancelGoal, "/spin/_action/cancel_goal"
        )
        self.navigate_cancel_client = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )
        self.controller_parameters = AsyncParameterClient(self, "/controller_server")
        self.zero_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            LaserScan, args.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, args.odom_topic, self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            Float32,
            args.battery_topic,
            self._on_battery,
            qos_profile_sensor_data,
        )
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            GoalStatusArray,
            "/spin/_action/status",
            self._on_spin_status,
            status_qos,
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._on_navigate_status,
            status_qos,
        )

    def _on_scan(self, message: LaserScan) -> None:
        with self._lock:
            self._scan = message
            self._scan_at = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        with self._lock:
            self._odom = message
            self._odom_at = time.monotonic()

    def _on_battery(self, message: Float32) -> None:
        with self._lock:
            self._battery = message
            self._battery_at = time.monotonic()

    def _on_spin_status(self, message: GoalStatusArray) -> None:
        with self._lock:
            self._spin_status = message

    def _on_navigate_status(self, message: GoalStatusArray) -> None:
        with self._lock:
            self._navigate_status = message

    def request_stop(self, reason: str) -> None:
        if not self.stop_requested.is_set():
            self.get_logger().warning(f"stop requested: {reason}")
            self.stop_requested.set()

    def _snapshot(self) -> tuple[Any, float, Any, float, Any, float]:
        with self._lock:
            return (
                self._scan,
                self._scan_at,
                self._odom,
                self._odom_at,
                self._battery,
                self._battery_at,
            )

    def _goal_is_active(self, status: Optional[GoalStatusArray]) -> bool:
        return bool(
            status
            and any(item.status in ACTIVE_GOAL_STATES for item in status.status_list)
        )

    def _wait_for_initial_data(self, timeout_s: float = 30.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self.stop_requested.is_set():
            scan, _, odom, _, battery, _ = self._snapshot()
            if scan is not None and odom is not None and battery is not None:
                return
            time.sleep(0.05)
        scan, _, odom, _, battery, _ = self._snapshot()
        missing = [
            name
            for name, value in (
                ("scan", scan),
                ("odometry", odom),
                ("battery", battery),
            )
            if value is None
        ]
        if self.stop_requested.is_set():
            raise UnsafeError("stop requested while waiting for initial sensor data")
        raise UnsafeError(
            "timed out waiting for initial ROS data; missing: " + ", ".join(missing)
        )

    def _lookup_transform(self, target: str, source: str) -> Any:
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as exc:
            raise UnsafeError(f"missing TF {target} <- {source}: {exc}") from exc

    def _scan_frame(self, scan: LaserScan) -> str:
        frame = scan.header.frame_id.strip().lstrip("/")
        if not frame:
            raise UnsafeError("LaserScan has an empty frame_id")
        return frame

    def _scan_clearance(self, scan: LaserScan) -> tuple[float, float, int]:
        if len(scan.ranges) < 90 or not math.isfinite(scan.angle_increment):
            raise UnsafeError("LaserScan geometry is invalid")
        coverage = abs(scan.angle_increment) * max(0, len(scan.ranges) - 1)
        if coverage < math.radians(350.0):
            raise UnsafeError(
                f"LaserScan covers only {math.degrees(coverage):.1f} degrees"
            )
        scan_frame = self._scan_frame(scan)
        if (
            self._base_from_scan is None
            or self._expected_scan_frame is None
            or scan_frame != self._expected_scan_frame
        ):
            raise UnsafeError("base-to-lidar transform was not preflighted")
        finite_clearances: list[float] = []
        usable = 0
        for index, value in enumerate(scan.ranges):
            if math.isinf(value) and value > 0.0:
                usable += 1
            elif (
                math.isfinite(value)
                and value >= max(0.0, scan.range_min)
                and value <= scan.range_max
            ):
                angle = scan.angle_min + index * scan.angle_increment
                point_base = _apply_transform(
                    self._base_from_scan,
                    (
                        float(value) * math.cos(angle),
                        float(value) * math.sin(angle),
                        0.0,
                    ),
                )
                # Clearance is measured from base_link, correcting the 60 mm lidar
                # origin offset instead of trusting raw polar range.
                finite_clearances.append(math.hypot(point_base[0], point_base[1]))
                usable += 1
        usable_fraction = usable / len(scan.ranges)
        finite_fraction = len(finite_clearances) / len(scan.ranges)
        if usable_fraction < 0.70 or finite_fraction < 0.70:
            raise UnsafeError(
                f"LaserScan health is insufficient ({usable_fraction:.0%} usable, "
                f"{finite_fraction:.0%} finite returns)"
            )
        return min(finite_clearances), coverage, len(finite_clearances)

    def _sensor_guard_reason(self, clearance_m: float) -> Optional[str]:
        now = time.monotonic()
        scan, scan_at, odom, odom_at, battery, battery_at = self._snapshot()
        if scan is None or now - scan_at > 0.75:
            return "scan is missing or stale"
        if odom is None or now - odom_at > 1.0:
            return "odometry is missing or stale"
        if battery is None or now - battery_at > 5.0:
            return "battery state is missing or stale"
        voltage = float(battery.data)
        if not math.isfinite(voltage) or voltage < self.args.min_battery_voltage:
            return f"battery voltage is unsafe ({voltage!r} V)"
        try:
            minimum, _, _ = self._scan_clearance(scan)
        except UnsafeError as exc:
            return str(exc)
        if minimum < clearance_m:
            return (
                f"lidar clearance {minimum:.3f} m is below required "
                f"{clearance_m:.3f} m"
            )
        return None

    def _fetch_health(self) -> dict[str, Any]:
        request = urllib.request.Request(
            self.args.yolo_url,
            headers={"Accept": "application/json", "Cache-Control": "no-store"},
        )
        try:
            with urllib.request.urlopen(request, timeout=1.0) as response:
                body = response.read(256 * 1024)
        except urllib.error.HTTPError as exc:
            # Degraded health is still JSON; parsing it lets us report the exact gate.
            body = exc.read(256 * 1024)
        except (OSError, urllib.error.URLError) as exc:
            raise UnsafeError(f"YOLO health request failed: {exc}") from exc
        try:
            payload = json.loads(body)
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise UnsafeError("YOLO health did not return valid JSON") from exc
        if not isinstance(payload, dict):
            raise UnsafeError("YOLO health JSON is not an object")
        return payload

    def _validate_health(self, payload: dict[str, Any]) -> tuple[int, int, int]:
        if payload.get("status") != "ok" or payload.get("detections_fresh") is not True:
            raise UnsafeError("YOLO pipeline is degraded or detections are stale")
        try:
            seq = int(payload["detection_seq"])
            width = int(payload["image_width"])
            height = int(payload["image_height"])
            age_ms = float(payload["detection_age_ms"])
            completed_ns = int(payload["detection_completed_wall_time_ns"])
        except (KeyError, TypeError, ValueError) as exc:
            raise UnsafeError("YOLO health lacks structured freshness fields") from exc
        if seq <= 0 or width < 160 or height < 120:
            raise UnsafeError("YOLO health sequence or image dimensions are invalid")
        if not math.isfinite(age_ms) or age_ms < 0.0:
            raise UnsafeError("YOLO detection age is invalid")
        wall_age_s = (time.time_ns() - completed_ns) / 1_000_000_000.0
        if (
            age_ms > self.args.max_detection_age_s * 1000.0
            or wall_age_s > self.args.max_detection_age_s
            or wall_age_s < -0.5
        ):
            raise UnsafeError(
                f"YOLO inference is stale or clock-invalid (age {wall_age_s:.2f} s)"
            )
        if not isinstance(payload.get("detections"), list):
            raise UnsafeError("YOLO health detections field is not a list")
        return seq, width, height

    def _wait_valid_yolo_health(self, timeout_s: float = 5.0) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        last_error = "no valid health sample"
        while time.monotonic() < deadline and not self.stop_requested.is_set():
            try:
                payload = self._fetch_health()
                self._validate_health(payload)
                return payload
            except UnsafeError as exc:
                last_error = str(exc)
                time.sleep(0.15)
        if self.stop_requested.is_set():
            raise UnsafeError("stop requested while waiting for YOLO health")
        raise UnsafeError(f"YOLO health did not recover: {last_error}")

    def _wait_yolo_advance(self, timeout_s: float = 8.0) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        first_payload = self._wait_valid_yolo_health(
            timeout_s=min(5.0, timeout_s)
        )
        first_seq, _, _ = self._validate_health(first_payload)
        last_error = "sequence did not advance"
        while time.monotonic() < deadline and not self.stop_requested.is_set():
            time.sleep(0.15)
            try:
                payload = self._fetch_health()
                seq, _, _ = self._validate_health(payload)
            except UnsafeError as exc:
                last_error = str(exc)
                continue
            if seq != first_seq:
                return payload
        raise UnsafeError(f"YOLO inference is not live: {last_error}")

    def _targets_from_health(
        self, payload: dict[str, Any]
    ) -> list[BottleObservation]:
        seq, width, height = self._validate_health(payload)
        target_class = self.args.target_class.lower()
        observations: list[BottleObservation] = []
        for item in payload["detections"]:
            if not isinstance(item, dict) or str(item.get("class", "")).lower() != target_class:
                continue
            try:
                confidence = float(item["confidence"])
                bbox_values = tuple(float(value) for value in item["bbox_xyxy"])
            except (KeyError, TypeError, ValueError) as exc:
                raise UnsafeError("YOLO target detection has a malformed bbox") from exc
            if len(bbox_values) != 4 or not all(
                math.isfinite(value) for value in bbox_values
            ):
                raise UnsafeError("YOLO target bbox is invalid")
            x1, y1, x2, y2 = bbox_values
            if not math.isfinite(confidence):
                raise UnsafeError("YOLO target confidence is non-finite")
            if (
                confidence < self.args.detection_confidence
                or x1 < 0.0
                or y1 < 0.0
                or x2 > width
                or y2 > height
                or x2 - x1 < 3.0
                or y2 - y1 < 3.0
            ):
                continue
            observations.append(
                BottleObservation(seq, confidence, bbox_values, width, height)
            )
        return observations

    def _confirm_target(
        self, baseline_seq: Optional[int], timeout_s: float
    ) -> Optional[BottleObservation]:
        deadline = time.monotonic() + timeout_s
        seen_sequences: set[int] = set()
        valid_sequences = 0
        track: list[BottleObservation] = []
        while time.monotonic() < deadline and not self.stop_requested.is_set():
            try:
                payload = self._fetch_health()
                seq, _, _ = self._validate_health(payload)
                if seq == baseline_seq or seq in seen_sequences:
                    time.sleep(0.10)
                    continue
                seen_sequences.add(seq)
                valid_sequences += 1
                targets = self._targets_from_health(payload)
            except UnsafeError as exc:
                self.get_logger().warning(f"YOLO stare sample rejected: {exc}")
                time.sleep(0.15)
                continue
            if not targets:
                track = []
            else:
                best = max(targets, key=lambda item: item.confidence)
                if track and _bbox_similar(track[-1], best):
                    track.append(best)
                else:
                    track = [best]
                if len(track) >= self.args.detection_confirmations:
                    confirmed = _average_observations(track)
                    self.get_logger().info(
                        f"confirmed {self.args.target_class} in {len(track)} distinct "
                        f"inference frames (confidence >= {confirmed.confidence:.2f})"
                    )
                    return confirmed
            time.sleep(0.10)
        if self.stop_requested.is_set():
            raise UnsafeError("stop requested while waiting for YOLO stare samples")
        if valid_sequences < self.args.detection_confirmations:
            raise UnsafeError(
                "insufficient fresh YOLO frames during stare; absence cannot be proven"
            )
        return None

    def _current_yolo_seq(self) -> int:
        payload = self._wait_valid_yolo_health()
        seq, _, _ = self._validate_health(payload)
        return seq

    def _current_search_yaw(self) -> float:
        # /hardware/odom reports velocity but leaves pose at identity on this rover.
        # The live odom -> base_link TF is the measured ICP/wheel-integrated yaw.
        transform = _ros_transform(
            self._lookup_transform(self.args.yaw_frame, self.args.base_frame)
        )
        return _yaw_from_quaternion(transform.quaternion)

    def _preflight(self) -> dict[str, Any]:
        self._wait_for_initial_data()
        scan, scan_at, odom, odom_at, battery, battery_at = self._snapshot()
        assert scan is not None and odom is not None and battery is not None
        now = time.monotonic()
        if now - scan_at > 0.75 or now - odom_at > 1.0 or now - battery_at > 5.0:
            raise UnsafeError("one or more preflight sensor streams are stale")
        voltage = float(battery.data)
        if not math.isfinite(voltage) or voltage < self.args.min_battery_voltage:
            raise UnsafeError(
                f"battery voltage {voltage!r} V is below "
                f"{self.args.min_battery_voltage:.2f} V"
            )
        linear_speed = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
        angular_speed = abs(odom.twist.twist.angular.z)
        if linear_speed > 0.03 or angular_speed > 0.08:
            raise UnsafeError(
                f"rover is already moving (linear={linear_speed:.3f} m/s, "
                f"angular={angular_speed:.3f} rad/s)"
            )
        scan_frame = self._scan_frame(scan)
        self._base_from_scan = _ros_transform(
            self._lookup_transform(self.args.base_frame, scan_frame)
        )
        self._expected_scan_frame = scan_frame
        self._lookup_transform(self.args.yaw_frame, self.args.base_frame)
        minimum, coverage, returns = self._scan_clearance(scan)
        if minimum < self.args.min_clearance_m:
            raise UnsafeError(
                f"nearest lidar return {minimum:.3f} m is below the full-circle "
                f"clearance gate {self.args.min_clearance_m:.3f} m"
            )
        if not self.spin_client.wait_for_server(timeout_sec=3.0):
            raise UnsafeError("Nav2 /spin action server is unavailable")
        if not self.args.search_only:
            if not self.navigate_client.wait_for_server(timeout_sec=3.0):
                raise UnsafeError("Nav2 /navigate_to_pose action server is unavailable")
            self._lookup_transform(self.args.map_frame, self.args.base_frame)
            if not self.controller_parameters.wait_for_service(timeout_sec=2.0):
                raise UnsafeError("controller_server parameter service is unavailable")
            parameter_future = self.controller_parameters.get_parameters(
                ["FollowPath.use_collision_detection"]
            )
            if not self._wait_future(parameter_future, 2.0):
                raise UnsafeError("controller collision parameter query timed out")
            parameter_response = parameter_future.result()
            parameter_values = (
                [] if parameter_response is None else parameter_response.values
            )
            if (
                not parameter_values
                or parameter_values[0].type != ParameterType.PARAMETER_BOOL
                or parameter_values[0].bool_value is not True
            ):
                raise UnsafeError(
                    "approach is locked: controller "
                    "FollowPath.use_collision_detection must be true"
                )
        with self._lock:
            if self._goal_is_active(self._spin_status) or self._goal_is_active(
                self._navigate_status
            ):
                raise UnsafeError("another Nav2 Spin or NavigateToPose goal is active")
        health = self._wait_yolo_advance()
        _, width, height = self._validate_health(health)
        self.get_logger().info(
            f"preflight passed: battery={voltage:.2f} V, clearance={minimum:.3f} m, "
            f"scan={math.degrees(coverage):.1f} deg/{returns} returns, "
            f"YOLO={width}x{height}"
        )
        return health

    def _wait_future(
        self, future: Any, timeout_s: float, *, ignore_stop: bool = False
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and (
            ignore_stop or not self.stop_requested.is_set()
        ):
            if future.done():
                return True
            time.sleep(0.025)
        return future.done()

    def _publish_zero(self, bursts: int = 5) -> None:
        message = Twist()
        for _ in range(bursts):
            self.zero_publisher.publish(message)
            time.sleep(0.04)

    def _wait_future_while_stopped(self, future: Any, timeout_s: float) -> bool:
        """Wait for action cleanup while continuously asserting zero velocity."""
        deadline = time.monotonic() + timeout_s
        message = Twist()
        while time.monotonic() < deadline:
            self.zero_publisher.publish(message)
            if future.done():
                return True
            time.sleep(0.05)
        return future.done()

    def _cancel_active_goal(self) -> None:
        with self._lock:
            handle = self._active_goal
            result_future = self._active_result_future
        if handle is not None:
            try:
                cancel_future = handle.cancel_goal_async()
                # Zero velocity immediately; cancellation still has to be confirmed.
                self._publish_zero(bursts=2)
                if not self._wait_future_while_stopped(cancel_future, 2.0):
                    self.get_logger().error("goal cancellation response timed out")
                else:
                    response = cancel_future.result()
                    if not response.goals_canceling and not (
                        result_future is not None and result_future.done()
                    ):
                        self.get_logger().error(
                            "action server did not acknowledge goal cancellation"
                        )
                if result_future is not None and not self._wait_future_while_stopped(
                    result_future, 3.0
                ):
                    self.get_logger().error(
                        "canceled goal did not reach a terminal action state"
                    )
            except Exception as exc:  # cancellation must never suppress zero velocity
                self.get_logger().error(f"goal cancellation raised: {exc}")
        self._publish_zero()
        with self._lock:
            self._active_goal = None
            self._active_result_future = None

    def _cancel_unknown_goal(self, client: Any, label: str) -> None:
        """Cancel one action scope if dispatch happened but acceptance is unknown."""
        self._publish_zero(bursts=2)
        if not client.wait_for_service(timeout_sec=0.75):
            self.get_logger().error(
                f"{label} cancel service unavailable after uncertain dispatch"
            )
            return
        try:
            # A default CancelGoal request (zero UUID and zero stamp) cancels all
            # goals for this action. Preflight already forbids an external goal;
            # canceling the action scope is the only fail-closed choice here.
            future = client.call_async(CancelGoal.Request())
            if not self._wait_future_while_stopped(future, 2.0):
                self.get_logger().error(
                    f"{label} cancel-all response timed out after uncertain dispatch"
                )
            elif future.result() is None:
                self.get_logger().error(
                    f"{label} cancel-all returned no response after uncertain dispatch"
                )
        except Exception as exc:
            self.get_logger().error(f"{label} cancel-all raised: {exc}")
        finally:
            self._publish_zero()

    def _execute_action(
        self,
        client: ActionClient,
        cancel_client: Any,
        goal: Any,
        timeout_s: float,
        label: str,
        guardian: Callable[[], Optional[str]],
    ) -> Any:
        if self.stop_requested.is_set():
            raise UnsafeError("stop was requested before action dispatch")
        send_future = client.send_goal_async(goal)
        acceptance_deadline = time.monotonic() + 3.0
        acceptance_failure: Optional[str] = None
        while not send_future.done():
            if self.stop_requested.is_set():
                acceptance_failure = "interrupted during goal acceptance"
                break
            reason = guardian()
            if reason:
                acceptance_failure = f"safety guardian during acceptance: {reason}"
                break
            if time.monotonic() >= acceptance_deadline:
                acceptance_failure = "goal acceptance timed out"
                break
            time.sleep(0.025)
        if acceptance_failure is not None or not send_future.done():
            self._cancel_unknown_goal(cancel_client, label)
            raise UnsafeError(f"{label} {acceptance_failure or 'acceptance is unknown'}")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise UnsafeError(f"{label} goal was rejected")
        result_future = handle.get_result_async()
        with self._lock:
            self._active_goal = handle
            self._active_result_future = result_future
        deadline = time.monotonic() + timeout_s
        try:
            while not result_future.done():
                if self.stop_requested.is_set():
                    raise UnsafeError(f"{label} interrupted")
                reason = guardian()
                if reason:
                    raise UnsafeError(f"{label} safety guardian: {reason}")
                if time.monotonic() >= deadline:
                    raise UnsafeError(f"{label} timed out after {timeout_s:.1f} s")
                time.sleep(0.05)
            wrapped_result = result_future.result()
            if wrapped_result is None or wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
                status = None if wrapped_result is None else wrapped_result.status
                raise UnsafeError(f"{label} ended with action status {status}")
            return wrapped_result.result
        except BaseException:
            self._cancel_active_goal()
            raise
        finally:
            with self._lock:
                self._active_goal = None
                self._active_result_future = None
            self._publish_zero(bursts=3)

    def _spin_step(self, radians: float, index: int, total: int) -> None:
        goal = Spin.Goal()
        goal.target_yaw = float(radians)
        goal.time_allowance = _duration_message(self.args.spin_timeout_s)
        self.get_logger().info(
            f"spin step {index}/{total}: {math.degrees(radians):.1f} degrees"
        )
        self._execute_action(
            self.spin_client,
            self.spin_cancel_client,
            goal,
            self.args.spin_timeout_s + 2.0,
            f"spin step {index}/{total}",
            lambda: self._sensor_guard_reason(self.args.guardian_clearance_m),
        )

    def _project_lidar_point(
        self, calibration: Calibration, point: Sequence[float]
    ) -> Optional[tuple[float, float]]:
        x_camera, y_camera, z_camera = _apply_transform(
            calibration.lidar_to_camera, point
        )
        if z_camera <= 0.05:
            return None
        intrinsics = calibration.intrinsics
        x = x_camera / z_camera
        y = y_camera / z_camera
        k1, k2, p1, p2, k3 = intrinsics.distortion
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        distorted_x = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        distorted_y = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        return (
            intrinsics.fx * distorted_x + intrinsics.cx,
            intrinsics.fy * distorted_y + intrinsics.cy,
        )

    def _associate_one_scan(
        self,
        scan: LaserScan,
        observation: BottleObservation,
        calibration: Calibration,
    ) -> LidarTarget:
        if (
            observation.width != calibration.intrinsics.width
            or observation.height != calibration.intrinsics.height
        ):
            raise UnsafeError(
                "YOLO image dimensions changed after calibration validation"
            )
        scan_frame = self._scan_frame(scan)
        if scan_frame != calibration.lidar_frame:
            raise UnsafeError(
                f"scan frame {scan_frame!r} does not match calibrated frame "
                f"{calibration.lidar_frame!r}"
            )
        base_from_lidar = _ros_transform(
            self._lookup_transform(self.args.base_frame, scan_frame)
        )
        x1, y1, x2, y2 = observation.bbox
        padding_x = (x2 - x1) * calibration.association.bbox_padding_fraction
        padding_y = (y2 - y1) * calibration.association.bbox_padding_fraction
        x1, x2 = max(0.0, x1 - padding_x), min(observation.width, x2 + padding_x)
        y1, y2 = max(0.0, y1 - padding_y), min(observation.height, y2 + padding_y)

        # (scan index, raw range, base x, base y, projected u)
        projected: list[tuple[int, float, float, float, float]] = []
        for index, raw_range in enumerate(scan.ranges):
            distance = float(raw_range)
            if (
                not math.isfinite(distance)
                or distance < max(scan.range_min, 0.03)
                or distance > scan.range_max
            ):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            lidar_point = (distance * math.cos(angle), distance * math.sin(angle), 0.0)
            pixel = self._project_lidar_point(calibration, lidar_point)
            if pixel is None or not (x1 <= pixel[0] <= x2 and y1 <= pixel[1] <= y2):
                continue
            x_base, y_base, _ = _apply_transform(base_from_lidar, lidar_point)
            projected.append((index, distance, x_base, y_base, pixel[0]))
        if len(projected) < calibration.association.min_lidar_points:
            raise UnsafeError(
                f"only {len(projected)} lidar points project into the target bbox"
            )

        projected.sort(key=lambda item: item[0])
        groups: list[list[tuple[int, float, float, float, float]]] = []
        for item in projected:
            if not groups or item[0] - groups[-1][-1][0] > calibration.association.max_index_gap:
                groups.append([item])
            else:
                groups[-1].append(item)
        if len(groups) > 1:
            circular_gap = groups[0][0][0] + len(scan.ranges) - groups[-1][-1][0]
            if circular_gap <= calibration.association.max_index_gap:
                groups[0] = groups[-1] + groups[0]
                groups.pop()

        candidates: list[tuple[float, LidarTarget]] = []
        bbox_center_x = (x1 + x2) * 0.5
        for group in groups:
            if len(group) < calibration.association.min_lidar_points:
                continue
            ranges = [item[1] for item in group]
            median_range = statistics.median(ranges)
            mad = statistics.median(abs(value - median_range) for value in ranges)
            spread = max(ranges) - min(ranges)
            if (
                mad > calibration.association.max_range_mad_m
                or spread > calibration.association.max_range_spread_m
            ):
                continue
            x_base = statistics.median(item[2] for item in group)
            y_base = statistics.median(item[3] for item in group)
            range_base = math.hypot(x_base, y_base)
            bearing = math.atan2(y_base, x_base)
            if (
                x_base <= 0.0
                or range_base < self.args.standoff_m
                or abs(bearing) > calibration.association.max_target_bearing_rad
            ):
                continue
            pixel_error = abs(statistics.median(item[4] for item in group) - bbox_center_x)
            score = pixel_error / max(1.0, x2 - x1) + 0.01 * range_base
            candidates.append(
                (score, LidarTarget(x_base, y_base, range_base, bearing, len(group)))
            )
        if not candidates:
            raise UnsafeError("no compact forward lidar cluster matches the target bbox")
        candidates.sort(key=lambda item: item[0])
        if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < 0.10:
            raise UnsafeError("lidar association is ambiguous between multiple clusters")
        return candidates[0][1]

    def _associate_stably(
        self, observation: BottleObservation, calibration: Calibration
    ) -> LidarTarget:
        deadline = time.monotonic() + 2.5
        last_scan_at = 0.0
        targets: list[LidarTarget] = []
        last_error = "no scan samples"
        while time.monotonic() < deadline and not self.stop_requested.is_set():
            scan, scan_at, _, _, _, _ = self._snapshot()
            if scan is None or scan_at == last_scan_at:
                time.sleep(0.03)
                continue
            last_scan_at = scan_at
            try:
                target = self._associate_one_scan(scan, observation, calibration)
            except UnsafeError as exc:
                last_error = str(exc)
                targets = []
                time.sleep(0.03)
                continue
            if targets:
                drift = math.hypot(
                    target.x_base - targets[-1].x_base,
                    target.y_base - targets[-1].y_base,
                )
                if drift > calibration.association.max_scan_target_drift_m:
                    targets = [target]
                else:
                    targets.append(target)
            else:
                targets = [target]
            if len(targets) >= calibration.association.min_scans:
                x_base = statistics.median(item.x_base for item in targets)
                y_base = statistics.median(item.y_base for item in targets)
                range_m = math.hypot(x_base, y_base)
                result = LidarTarget(
                    x_base,
                    y_base,
                    range_m,
                    math.atan2(y_base, x_base),
                    min(item.points for item in targets),
                )
                self.get_logger().info(
                    f"lidar association stable across {len(targets)} scans: "
                    f"range={result.range_m:.2f} m, "
                    f"bearing={math.degrees(result.bearing_rad):.1f} deg"
                )
                return result
            time.sleep(0.03)
        raise UnsafeError(f"lidar association was not stable: {last_error}")

    def _navigate_to_target(self, target: LidarTarget) -> None:
        approach_distance = target.range_m - self.args.standoff_m
        if approach_distance <= 0.08:
            self.get_logger().info("already within configured target standoff")
            return
        if approach_distance > self.args.max_approach_m:
            raise UnsafeError(
                f"required approach {approach_distance:.2f} m exceeds "
                f"limit {self.args.max_approach_m:.2f} m"
            )
        map_from_base = _ros_transform(
            self._lookup_transform(self.args.map_frame, self.args.base_frame)
        )
        goal_in_base = (
            approach_distance * math.cos(target.bearing_rad),
            approach_distance * math.sin(target.bearing_rad),
            0.0,
        )
        goal_x, goal_y, _ = _apply_transform(map_from_base, goal_in_base)
        base_yaw = _yaw_from_quaternion(map_from_base.quaternion)
        goal_yaw = base_yaw + target.bearing_rad

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.z = math.sin(goal_yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(goal_yaw * 0.5)
        self.get_logger().info(
            f"sending standoff goal: approach={approach_distance:.2f} m, "
            f"map=({goal_x:.2f}, {goal_y:.2f})"
        )
        self._execute_action(
            self.navigate_client,
            self.navigate_cancel_client,
            goal,
            self.args.goal_timeout_s,
            "target approach",
            lambda: self._sensor_guard_reason(self.args.guardian_clearance_m),
        )

    def emergency_stop(self) -> None:
        self.stop_requested.set()
        self._cancel_active_goal()

    def run(self) -> int:
        health = self._preflight()
        _, image_width, image_height = self._validate_health(health)
        calibration: Optional[Calibration] = None
        if not self.args.search_only:
            calibration = load_calibration(
                self.args.calibration, image_width, image_height
            )
            if self.args.guardian_clearance_m < calibration.swept_radius_m + 0.10:
                raise UnsafeError(
                    "guardian clearance must exceed measured chassis swept radius "
                    "by at least 0.10 m"
                )
            self.get_logger().info(
                f"validated approach calibration for {calibration.lidar_frame} -> "
                f"{calibration.camera_frame}"
            )
        if self.args.dry_run:
            self.get_logger().info(
                "dry-run complete: all preflight gates passed; no action was sent"
            )
            return EXIT_OK

        steps = math.ceil(360.0 / self.args.step_deg)
        step_radians = 2.0 * math.pi / steps
        stare_timeout = max(3.0, self.args.detection_confirmations * 1.25)
        saw_visual_target = False

        baseline_seq = self._current_yolo_seq()
        observation = self._confirm_target(baseline_seq, stare_timeout)
        if observation is not None:
            saw_visual_target = True
            if self.args.search_only:
                return EXIT_OK
            assert calibration is not None
            try:
                target = self._associate_stably(observation, calibration)
                self._navigate_to_target(target)
                return EXIT_OK
            except UnsafeError as exc:
                self.get_logger().warning(f"initial target rejected for approach: {exc}")

        completed_steps = 0
        measured_rotation = 0.0
        previous_yaw = self._current_search_yaw()
        for index in range(1, steps + 1):
            reason = self._sensor_guard_reason(self.args.guardian_clearance_m)
            if reason:
                raise UnsafeError(f"before spin step {index}: {reason}")
            self._spin_step(step_radians, index, steps)
            completed_steps += 1
            # Let odometry and the camera settle, then require post-stop inference frames.
            time.sleep(0.35)
            current_yaw = self._current_search_yaw()
            measured_step = math.atan2(
                math.sin(current_yaw - previous_yaw),
                math.cos(current_yaw - previous_yaw),
            )
            previous_yaw = current_yaw
            if measured_step < step_radians * 0.35 or measured_step > step_radians * 2.5:
                raise UnsafeError(
                    f"spin step {index} odometry changed by an implausible "
                    f"{math.degrees(measured_step):.1f} degrees"
                )
            measured_rotation += measured_step
            baseline_seq = self._current_yolo_seq()
            observation = self._confirm_target(baseline_seq, stare_timeout)
            if observation is None:
                continue
            saw_visual_target = True
            if self.args.search_only:
                return EXIT_OK
            assert calibration is not None
            try:
                target = self._associate_stably(observation, calibration)
                self._navigate_to_target(target)
                return EXIT_OK
            except UnsafeError as exc:
                self.get_logger().warning(
                    f"target at search step {index} rejected for approach: {exc}"
                )

        if completed_steps != steps:
            raise UnsafeError("search ended before a complete 360 degree spin")
        minimum_full_rotation = 2.0 * math.pi - max(step_radians, math.radians(10.0))
        if measured_rotation < minimum_full_rotation:
            raise UnsafeError(
                f"search actions completed, but odometry measured only "
                f"{math.degrees(measured_rotation):.1f} degrees of rotation"
            )
        if saw_visual_target:
            raise UnsafeError(
                "target was visually detected, but no safe lidar association was found"
            )
        self.get_logger().info(
            "completed a full 360 degree search with no confirmed target "
            f"(odometry={math.degrees(measured_rotation):.1f} degrees)"
        )
        return EXIT_NOT_FOUND


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = SafeArgumentParser(
        description="Guarded Nav2 object search with calibration-locked approach"
    )
    parser.add_argument(
        "--target-class",
        required=True,
        help="YOLO/COCO class name to search for, e.g. bottle, chair, cup",
    )
    parser.add_argument("--dry-run", action="store_true", help="preflight only")
    parser.add_argument(
        "--search-only", action="store_true", help="find the target but never approach"
    )
    parser.add_argument(
        "--yolo-url", default="http://z-boys-yolo-live:8091/health"
    )
    parser.add_argument(
        "--calibration", default="/app/search_calibration.json"
    )
    parser.add_argument("--step-deg", type=float, default=15.0)
    parser.add_argument("--min-clearance-m", type=float, default=0.35)
    parser.add_argument("--guardian-clearance-m", type=float, default=0.10)
    parser.add_argument("--min-battery-voltage", type=float, default=6.8)
    parser.add_argument("--detection-confidence", type=float, default=0.60)
    parser.add_argument("--detection-confirmations", type=int, default=2)
    parser.add_argument("--max-detection-age-s", type=float, default=1.5)
    parser.add_argument("--standoff-m", type=float, default=0.60)
    parser.add_argument("--max-approach-m", type=float, default=1.50)
    parser.add_argument("--spin-timeout-s", type=float, default=20.0)
    parser.add_argument("--goal-timeout-s", type=float, default=60.0)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/hardware/odom")
    parser.add_argument("--battery-topic", default="/hardware/battery")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--yaw-frame", default="odom")
    args = parser.parse_args(argv)

    numeric_checks = (
        (5.0 <= args.step_deg <= 45.0, "--step-deg must be between 5 and 45"),
        (
            0.1 <= args.guardian_clearance_m <= 1.0,
            "--guardian-clearance-m must be between 0.25 and 1.0",
        ),
        (
            args.min_clearance_m >= args.guardian_clearance_m,
            "--min-clearance-m must be >= --guardian-clearance-m",
        ),
        (
            5.0 <= args.min_battery_voltage <= 12.0,
            "--min-battery-voltage must be between 5 and 12",
        ),
        (
            0.25 <= args.detection_confidence <= 0.95,
            "--detection-confidence must be between 0.25 and 0.95",
        ),
        (
            2 <= args.detection_confirmations <= 5,
            "--detection-confirmations must be between 2 and 5",
        ),
        (
            0.5 <= args.max_detection_age_s <= 3.0,
            "--max-detection-age-s must be between 0.5 and 3.0",
        ),
        (0.30 <= args.standoff_m <= 1.5, "--standoff-m must be 0.30 to 1.5"),
        (
            0.10 <= args.max_approach_m <= 3.0,
            "--max-approach-m must be 0.10 to 3.0",
        ),
        (
            5.0 <= args.spin_timeout_s <= 60.0,
            "--spin-timeout-s must be between 5 and 60",
        ),
        (
            10.0 <= args.goal_timeout_s <= 180.0,
            "--goal-timeout-s must be between 10 and 180",
        ),
    )
    for valid, message in numeric_checks:
        if not valid:
            parser.error(message)
    if not args.yolo_url.startswith("http://"):
        parser.error("--yolo-url must be an http:// URL on the rover network")
    args.target_class = args.target_class.strip()
    if not args.target_class or "/" in args.target_class:
        parser.error("--target-class must be a non-empty class name")
    for attribute in ("scan_topic", "odom_topic", "battery_topic"):
        if not getattr(args, attribute).startswith("/"):
            parser.error(f"--{attribute.replace('_', '-')} must be an absolute topic")
    for attribute in ("base_frame", "map_frame", "yaw_frame"):
        value = getattr(args, attribute).strip().lstrip("/")
        if not value:
            parser.error(f"--{attribute.replace('_', '-')} cannot be empty")
        setattr(args, attribute, value)
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    rclpy.init(args=None)
    node = BottleSearchNode(args)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin, name="object-search-ros", daemon=True
    )
    executor_thread.start()

    previous_handlers: dict[int, Any] = {}

    def handle_signal(signum: int, _frame: Any) -> None:
        node.request_stop(signal.Signals(signum).name)

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, handle_signal)

    exit_code = EXIT_ERROR
    try:
        exit_code = node.run()
    except UnsafeError as exc:
        node.get_logger().error(f"refusing to continue: {exc}")
    except KeyboardInterrupt:
        node.request_stop("keyboard interrupt")
    except BaseException as exc:
        node.get_logger().error(f"unexpected failure: {type(exc).__name__}: {exc}")
    finally:
        node.emergency_stop()
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
        executor.shutdown(timeout_sec=3.0)
        executor_thread.join(timeout=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
