#!/usr/bin/env python3
"""Autonomous vacuum-style frontier explorer for RoboMarvel.

Cartographer owns SLAM and continuously publishes ``/map``.  This node owns the
high-level exploration sequence:

1. wait for a fresh lidar scan and the Collision Monitor velocity input;
2. drive forward briefly to create the first useful baseline;
3. perform a full in-place lidar sweep;
4. select reachable frontiers and delegate obstacle-aware motion to Nav2;
5. perform one additional sweep before declaring the map complete.

Direct bootstrap motion is published to ``/cmd_vel_nav`` and therefore still
passes through Nav2 Collision Monitor before reaching ``/cmd_vel``.
"""
from __future__ import annotations

from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import queue
from pathlib import Path
import sys
import threading
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))
from frontier_core import GridSpec, FrontierGoal, select_frontier_goals  # noqa: E402


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


@dataclass
class GoalRecord:
    x: float
    y: float
    sent_monotonic: float
    description: str


class MazeExplorer(Node):
    def __init__(self) -> None:
        super().__init__("maze_explorer")

        defaults = {
            "map_topic": "/map",
            "scan_topic": "/scan",
            "navigate_action": "/navigate_to_pose",
            "global_frame": "map",
            "robot_frame": "base_link",
            # Never bypass Collision Monitor.  It consumes cmd_vel_nav and is
            # the only autonomous node that publishes the hardware /cmd_vel.
            "cmd_vel_topic": "/cmd_vel_nav",
            "plan_period_sec": 1.0,
            "goal_timeout_sec": 75.0,
            "goal_cooldown_sec": 0.6,
            "robot_radius_m": 0.11,
            "safety_margin_m": 0.12,
            "min_frontier_size_m": 0.25,
            "min_goal_distance_m": 0.30,
            "blacklist_radius_m": 0.55,
            "gain_weight": 1.2,
            "free_threshold": 20,
            "occupied_threshold": 65,
            "completion_cycles": 4,
            # Vacuum-style initial mapping motion.
            "bootstrap_forward_speed_mps": 0.12,
            "bootstrap_forward_duration_sec": 2.2,
            "bootstrap_front_clearance_m": 0.48,
            "bootstrap_pause_sec": 0.35,
            "scan_angular_speed_rps": 0.55,
            "scan_target_angle_rad": 6.05,
            "scan_timeout_sec": 9.0,
            "scan_stale_timeout_sec": 0.8,
            "front_sector_half_angle_rad": 0.40,
            "scan_mount_yaw_fallback_rad": 1.5708,
            "completion_rescan": True,
            "http_host": "0.0.0.0",
            "http_port": 8090,
            "start_enabled": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.action_name = str(self.get_parameter("navigate_action").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.plan_period = float(self.get_parameter("plan_period_sec").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self.goal_cooldown = float(self.get_parameter("goal_cooldown_sec").value)
        self.robot_radius = float(self.get_parameter("robot_radius_m").value)
        self.safety_margin = float(self.get_parameter("safety_margin_m").value)
        self.min_frontier_size = float(self.get_parameter("min_frontier_size_m").value)
        self.min_goal_distance = float(self.get_parameter("min_goal_distance_m").value)
        self.blacklist_radius = float(self.get_parameter("blacklist_radius_m").value)
        self.gain_weight = float(self.get_parameter("gain_weight").value)
        self.free_threshold = int(self.get_parameter("free_threshold").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.completion_cycles = int(self.get_parameter("completion_cycles").value)

        self.bootstrap_speed = float(
            self.get_parameter("bootstrap_forward_speed_mps").value
        )
        self.bootstrap_duration = float(
            self.get_parameter("bootstrap_forward_duration_sec").value
        )
        self.bootstrap_clearance = float(
            self.get_parameter("bootstrap_front_clearance_m").value
        )
        self.bootstrap_pause = float(
            self.get_parameter("bootstrap_pause_sec").value
        )
        self.scan_speed = float(self.get_parameter("scan_angular_speed_rps").value)
        self.scan_target_default = float(
            self.get_parameter("scan_target_angle_rad").value
        )
        self.scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self.scan_stale_timeout = float(
            self.get_parameter("scan_stale_timeout_sec").value
        )
        self.front_sector_half_angle = float(
            self.get_parameter("front_sector_half_angle_rad").value
        )
        self.scan_mount_yaw_fallback = float(
            self.get_parameter("scan_mount_yaw_fallback_rad").value
        )
        self.completion_rescan = bool(self.get_parameter("completion_rescan").value)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._on_map, map_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(String, "/maze/status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.start_service = self.create_service(
            Trigger, "/maze/start", self._start_service
        )
        self.stop_service = self.create_service(
            Trigger, "/maze/stop", self._stop_service
        )
        self.status_service = self.create_service(
            Trigger, "/maze/get_status", self._status_service
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = ActionClient(self, NavigateToPose, self.action_name)

        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_scan: Optional[LaserScan] = None
        self.last_scan_monotonic = 0.0
        self.front_clearance_m: Optional[float] = None

        self.enabled = bool(self.get_parameter("start_enabled").value)
        self.state = "IDLE"
        self.message = "Ожидание команды"
        self.frontier_count = 0
        self.no_frontier_cycles = 0
        self.completed_goals = 0
        self.failed_goals = 0
        self.blacklist: list[tuple[float, float]] = []
        self.current_goal: Optional[GoalRecord] = None
        self.goal_handle = None
        self.goal_request_pending = False
        self.stop_requested = False
        self.command_epoch = 0
        self.last_goal_finished = 0.0
        self.last_map_stamp = None

        # Direct motion state used only for the initial and completion sweeps.
        self.motion_phase: Optional[str] = None
        self.motion_phase_started: Optional[float] = None
        self.motion_reason = ""
        self.scan_target_angle = self.scan_target_default
        self.scan_accumulated_angle = 0.0
        self.scan_previous_yaw: Optional[float] = None
        self.scan_direction = 1.0
        self.bootstrap_complete = False
        self.completion_rescan_done = False

        self.http_commands: queue.Queue[str] = queue.Queue()
        self.status_lock = threading.Lock()

        self.create_timer(max(0.25, self.plan_period), self._plan_tick)
        self.create_timer(0.05, self._motion_tick)
        self.create_timer(0.1, self._process_http_commands)
        self.create_timer(0.5, self._publish_status)
        self.create_timer(0.25, self._watchdog)

        self.http_server = self._start_http_server()
        self.get_logger().info(
            "Maze explorer ready: original host UI uses /maze/start and /maze/stop; "
            f"API on port {self.get_parameter('http_port').value}"
        )
        if self.enabled:
            self._start_exploration(force=True)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        self.last_map_stamp = msg.header.stamp
        if self.enabled and self.bootstrap_complete and self.motion_phase is None:
            if self.state in ("WAITING_FOR_MAP", "WAITING_FOR_TF"):
                self.state = "PLANNING"
                self.message = "Карта получена, поиск границы"

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.last_scan_monotonic = time.monotonic()
        self.front_clearance_m = self._front_clearance(msg)

    def _front_clearance(self, msg: LaserScan) -> Optional[float]:
        # Convert each ray angle from the lidar frame into base_link.  The
        # physical sensor is mounted at +90 degrees in robot.urdf, so using raw
        # LaserScan angle zero as "forward" would inspect the wrong side.
        mount_yaw = self.scan_mount_yaw_fallback
        frame_id = msg.header.frame_id.strip()
        if frame_id:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame,
                    frame_id,
                    Time(),
                    timeout=Duration(seconds=0.03),
                )
                mount_yaw = yaw_from_quaternion(transform.transform.rotation)
            except TransformException:
                pass

        best = math.inf
        angle = float(msg.angle_min)
        for value in msg.ranges:
            base_angle = normalize_angle(mount_yaw + angle)
            if abs(base_angle) <= self.front_sector_half_angle:
                if math.isfinite(value) and msg.range_min <= value <= msg.range_max:
                    best = min(best, float(value))
            angle += float(msg.angle_increment)
        return None if math.isinf(best) else best

    def _start_service(self, _request, response):
        changed = self._start_exploration()
        response.success = True
        response.message = (
            "Автономное исследование запущено"
            if changed
            else "Исследование уже запущено"
        )
        return response

    def _stop_service(self, _request, response):
        self._stop_exploration("Остановлено оператором")
        response.success = True
        response.message = "Остановка запрошена"
        return response

    def _status_service(self, _request, response):
        response.success = True
        response.message = json.dumps(self._status_dict(), ensure_ascii=False)
        return response

    def _start_exploration(self, force: bool = False) -> bool:
        if self.enabled and not force:
            self.message = "Исследование уже запущено"
            return False
        self.command_epoch += 1
        self.enabled = True
        self.stop_requested = False
        self.blacklist.clear()
        self.no_frontier_cycles = 0
        self.completed_goals = 0
        self.failed_goals = 0
        self.frontier_count = 0
        self.completion_rescan_done = False
        self.bootstrap_complete = False
        self._cancel_goal()
        self._set_motion_phase("forward", "Начальный проезд для построения карты")
        self.get_logger().info("Исследование лабиринта запущено")
        return True

    def _stop_exploration(self, reason: str) -> None:
        self.command_epoch += 1
        self.enabled = False
        self.stop_requested = True
        self.state = "STOPPED"
        self.message = reason
        self.motion_phase = None
        self.motion_phase_started = None
        self._cancel_goal()
        self._publish_zero_velocity()
        self.get_logger().warning(reason)

    def _cancel_goal(self) -> None:
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f"Goal cancellation failed: {exc}")
        self.goal_handle = None
        self.current_goal = None
        self.goal_request_pending = False

    def _publish_velocity(self, linear: float = 0.0, angular: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def _publish_zero_velocity(self) -> None:
        self._publish_velocity()

    def _velocity_path_ready(self) -> bool:
        # At least one subscriber must exist on cmd_vel_nav.  In the intended
        # topology that subscriber is Collision Monitor.
        return self.count_subscribers(self.cmd_vel_topic) > 0

    def _scan_is_fresh(self) -> bool:
        return (
            self.latest_scan is not None
            and time.monotonic() - self.last_scan_monotonic <= self.scan_stale_timeout
        )

    def _set_motion_phase(
        self,
        phase: str,
        reason: str,
        *,
        scan_target_angle: Optional[float] = None,
    ) -> None:
        self.motion_phase = phase
        self.motion_phase_started = None
        self.motion_reason = reason
        if scan_target_angle is not None:
            self.scan_target_angle = float(scan_target_angle)
        if phase == "scan":
            self.scan_accumulated_angle = 0.0
            self.scan_previous_yaw = None
        state_names = {
            "forward": "BOOTSTRAP_FORWARD",
            "pause": "BOOTSTRAP_PAUSE",
            "scan": "SCANNING",
        }
        self.state = state_names.get(phase, "MOTION")
        self.message = reason

    def _begin_scan(self, reason: str, target_angle: Optional[float] = None) -> None:
        self.scan_direction *= -1.0
        self._set_motion_phase(
            "scan",
            reason,
            scan_target_angle=target_angle or self.scan_target_default,
        )

    def _finish_motion_sequence(self) -> None:
        self._publish_zero_velocity()
        self.motion_phase = None
        self.motion_phase_started = None
        self.bootstrap_complete = True
        if self.latest_map is None:
            self.state = "WAITING_FOR_MAP"
            self.message = "Начальный обзор завершён, ожидание /map"
        else:
            self.state = "PLANNING"
            self.message = "Обзор завершён, поиск следующей области"

    def _motion_tick(self) -> None:
        if not self.enabled or self.motion_phase is None:
            return

        if not self._velocity_path_ready():
            self._publish_zero_velocity()
            self.state = "WAITING_FOR_SAFETY"
            self.message = "Ожидание Collision Monitor на /cmd_vel_nav"
            self.motion_phase_started = None
            return

        if not self._scan_is_fresh():
            self._publish_zero_velocity()
            self.state = "WAITING_FOR_SCAN"
            self.message = "Ожидание свежего /scan"
            self.motion_phase_started = None
            return

        now = time.monotonic()
        if self.motion_phase_started is None:
            self.motion_phase_started = now
            if self.motion_phase == "scan":
                pose = self._robot_pose(report_error=False)
                self.scan_previous_yaw = None if pose is None else pose[2]
        elapsed = now - self.motion_phase_started

        if self.motion_phase == "forward":
            clearance = self.front_clearance_m
            if clearance is not None and clearance < self.bootstrap_clearance:
                self._publish_zero_velocity()
                self._set_motion_phase(
                    "pause",
                    f"Препятствие впереди ({clearance:.2f} м), переход к обзору",
                )
                return
            if elapsed >= self.bootstrap_duration:
                self._publish_zero_velocity()
                self._set_motion_phase("pause", "Короткая остановка перед обзором")
                return
            self.state = "BOOTSTRAP_FORWARD"
            suffix = "" if clearance is None else f", впереди {clearance:.2f} м"
            self.message = (
                f"Начальный проезд {elapsed:.1f}/{self.bootstrap_duration:.1f} с"
                f"{suffix}"
            )
            self._publish_velocity(linear=self.bootstrap_speed)
            return

        if self.motion_phase == "pause":
            self._publish_zero_velocity()
            self.state = "BOOTSTRAP_PAUSE"
            self.message = "Остановка перед круговым сканированием"
            if elapsed >= self.bootstrap_pause:
                self._begin_scan("Круговое сканирование лидаром")
            return

        if self.motion_phase == "scan":
            pose = self._robot_pose(report_error=False)
            if pose is not None:
                yaw = pose[2]
                if self.scan_previous_yaw is not None:
                    self.scan_accumulated_angle += abs(
                        normalize_angle(yaw - self.scan_previous_yaw)
                    )
                self.scan_previous_yaw = yaw

            self.state = "SCANNING"
            degrees = math.degrees(self.scan_accumulated_angle)
            target_degrees = math.degrees(self.scan_target_angle)
            self.message = f"Сканирование: {degrees:.0f}/{target_degrees:.0f}°"
            self._publish_velocity(angular=self.scan_direction * self.scan_speed)

            if (
                self.scan_accumulated_angle >= self.scan_target_angle
                or elapsed >= self.scan_timeout
            ):
                self._finish_motion_sequence()
            return

    def _robot_pose(self, report_error: bool = True) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=0.3),
            )
        except TransformException as exc:
            if report_error:
                self.state = "WAITING_FOR_TF"
                self.message = f"Нет TF map→base_link: {exc}"
            return None
        t = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return t.x, t.y, yaw

    def _grid_spec(self, msg: OccupancyGrid) -> GridSpec:
        origin = msg.info.origin
        return GridSpec(
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution=float(msg.info.resolution),
            origin_x=float(origin.position.x),
            origin_y=float(origin.position.y),
            origin_yaw=yaw_from_quaternion(origin.orientation),
        )

    def _plan_tick(self) -> None:
        if (
            not self.enabled
            or not self.bootstrap_complete
            or self.motion_phase is not None
            or self.goal_request_pending
            or self.goal_handle is not None
        ):
            return
        if time.monotonic() - self.last_goal_finished < self.goal_cooldown:
            return
        if self.latest_map is None:
            self.state = "WAITING_FOR_MAP"
            self.message = "Ожидание /map"
            return
        if not self.navigator.wait_for_server(timeout_sec=0.2):
            self.state = "WAITING_FOR_NAV2"
            self.message = "Ожидание активного Nav2 NavigateToPose"
            return

        pose = self._robot_pose()
        if pose is None:
            return
        map_msg = self.latest_map
        goals = select_frontier_goals(
            spec=self._grid_spec(map_msg),
            data=map_msg.data,
            robot_pose=pose,
            inflation_radius_m=self.robot_radius + self.safety_margin,
            min_frontier_size_m=self.min_frontier_size,
            min_goal_distance_m=self.min_goal_distance,
            free_threshold=self.free_threshold,
            occupied_threshold=self.occupied_threshold,
            gain_weight=self.gain_weight,
            blacklist=self.blacklist,
            blacklist_radius_m=self.blacklist_radius,
        )
        self.frontier_count = len(goals)
        if not goals:
            self.no_frontier_cycles += 1
            self.state = "VERIFYING_COMPLETE"
            self.message = (
                f"Доступных границ нет: проверка "
                f"{self.no_frontier_cycles}/{self.completion_cycles}"
            )
            # A vacuum-style final sweep often reveals doors and side areas that
            # were hidden from the last frontier pose.
            if (
                self.completion_rescan
                and not self.completion_rescan_done
                and self.no_frontier_cycles >= 2
            ):
                self.completion_rescan_done = True
                self.no_frontier_cycles = 0
                self._begin_scan("Контрольное круговое сканирование")
                return
            if self.no_frontier_cycles >= self.completion_cycles:
                self.enabled = False
                self.state = "COMPLETE"
                self.message = "Все доступные области исследованы"
                self._publish_zero_velocity()
                self.get_logger().info(self.message)
            return

        self.no_frontier_cycles = 0
        self.completion_rescan_done = False
        self._send_goal(goals[0])

    def _send_goal(self, frontier: FrontierGoal) -> None:
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.global_frame
        pose.pose.position.x = frontier.world_x
        pose.pose.position.y = frontier.world_y
        qx, qy, qz, qw = quaternion_from_yaw(frontier.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        description = (
            f"frontier=({frontier.world_x:.2f}, {frontier.world_y:.2f}), "
            f"path≈{frontier.path_distance_m:.2f} m, "
            f"size≈{frontier.frontier_size_m:.2f} m"
        )
        self.current_goal = GoalRecord(
            frontier.world_x, frontier.world_y, time.monotonic(), description
        )
        self.goal_request_pending = True
        self.state = "SENDING_GOAL"
        self.message = f"Отправка цели: {description}"
        epoch = self.command_epoch
        future = self.navigator.send_goal_async(
            goal_msg, feedback_callback=self._feedback
        )
        future.add_done_callback(
            lambda result, sent_epoch=epoch: self._goal_response(result, sent_epoch)
        )

    def _goal_response(self, future, epoch: int) -> None:
        self.goal_request_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            if epoch == self.command_epoch:
                self._goal_failed(f"Ошибка отправки цели: {exc}")
            return
        if epoch != self.command_epoch or not self.enabled:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._goal_failed("Nav2 отклонил цель")
            return
        self.goal_handle = handle
        self.state = "NAVIGATING"
        self.message = (
            f"Движение к "
            f"{self.current_goal.description if self.current_goal else 'границе'}"
        )
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result, sent_epoch=epoch: self._goal_result(result, sent_epoch)
        )

    def _feedback(self, feedback_msg) -> None:
        try:
            remaining = float(feedback_msg.feedback.distance_remaining)
        except Exception:
            return
        if math.isfinite(remaining):
            self.message = f"Навигация: осталось примерно {remaining:.2f} м"

    def _goal_result(self, future, epoch: int) -> None:
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:
            if epoch == self.command_epoch:
                self._goal_failed(f"Ошибка результата Nav2: {exc}")
            return

        if epoch != self.command_epoch:
            return

        record = self.current_goal
        self.goal_handle = None
        self.current_goal = None
        self.last_goal_finished = time.monotonic()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.completed_goals += 1
            self.state = "PLANNING"
            self.message = "Граница достигнута, обновление карты"
            return
        if status == GoalStatus.STATUS_CANCELED and self.stop_requested:
            return
        if record is not None:
            self.blacklist.append((record.x, record.y))
        self.failed_goals += 1
        self.state = "RECOVERING"
        self.message = f"Цель недоступна (status={status}), выбрана другая"
        self.get_logger().warning(self.message)

    def _goal_failed(self, reason: str) -> None:
        if self.current_goal is not None:
            self.blacklist.append((self.current_goal.x, self.current_goal.y))
        self.failed_goals += 1
        self.goal_handle = None
        self.current_goal = None
        self.goal_request_pending = False
        self.last_goal_finished = time.monotonic()
        self.state = "RECOVERING"
        self.message = reason
        self.get_logger().warning(reason)

    def _watchdog(self) -> None:
        if not self.enabled or self.current_goal is None:
            return
        elapsed = time.monotonic() - self.current_goal.sent_monotonic
        if elapsed <= self.goal_timeout:
            return
        record = self.current_goal
        self.blacklist.append((record.x, record.y))
        self.failed_goals += 1
        self.message = f"Тайм-аут цели {elapsed:.0f} с; цель заблокирована"
        self.state = "RECOVERING"
        self._cancel_goal()
        self._publish_zero_velocity()
        self.last_goal_finished = time.monotonic()
        self.get_logger().warning(self.message)

    def _status_dict(self) -> dict:
        with self.status_lock:
            goal = None
            if self.current_goal is not None:
                goal = {
                    "x": round(self.current_goal.x, 3),
                    "y": round(self.current_goal.y, 3),
                    "elapsed_sec": round(
                        time.monotonic() - self.current_goal.sent_monotonic, 1
                    ),
                }
            return {
                "active": self.enabled,
                "state": self.state,
                "message": self.message,
                "map_received": self.latest_map is not None,
                "scan_fresh": self._scan_is_fresh(),
                "front_clearance_m": (
                    None
                    if self.front_clearance_m is None
                    else round(self.front_clearance_m, 3)
                ),
                "motion_phase": self.motion_phase,
                "scan_angle_deg": round(math.degrees(self.scan_accumulated_angle), 1),
                "frontier_candidates": self.frontier_count,
                "completed_goals": self.completed_goals,
                "failed_goals": self.failed_goals,
                "blacklisted_goals": len(self.blacklist),
                "goal": goal,
            }

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self._status_dict(), ensure_ascii=False)
        self.status_pub.publish(msg)

    def _process_http_commands(self) -> None:
        while True:
            try:
                command = self.http_commands.get_nowait()
            except queue.Empty:
                return
            if command == "start":
                self._start_exploration()
            elif command == "stop":
                self._stop_exploration("Остановлено кнопкой интерфейса")

    def _start_http_server(self):
        host = str(self.get_parameter("http_host").value)
        port = int(self.get_parameter("http_port").value)
        explorer = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "RoboMarvelMazeAPI/2.0"

            def _json(self, payload: dict, status=HTTPStatus.OK) -> None:
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):  # noqa: N802
                if self.path == "/api/maze/status":
                    self._json(explorer._status_dict())
                    return
                if self.path in ("/", "/health"):
                    self._json(
                        {
                            "service": "RoboMarvel maze API",
                            "ui": "Use the original RoboMarvel home page on port 80",
                            "status_endpoint": "/api/maze/status",
                        }
                    )
                    return
                self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

            def do_POST(self):  # noqa: N802
                if self.path == "/api/maze/start":
                    explorer.http_commands.put("start")
                    self._json({"ok": True, "message": "Запуск принят"})
                    return
                if self.path == "/api/maze/stop":
                    explorer.http_commands.put("stop")
                    self._json({"ok": True, "message": "Остановка принята"})
                    return
                self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

            def log_message(self, fmt, *args):
                explorer.get_logger().debug("HTTP " + fmt % args)

        try:
            server = ThreadingHTTPServer((host, port), Handler)
        except OSError as exc:
            self.get_logger().error(f"Could not start maze API on {host}:{port}: {exc}")
            return None
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        return server

    def destroy_node(self):
        self._publish_zero_velocity()
        if self.http_server is not None:
            self.http_server.shutdown()
            self.http_server.server_close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MazeExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
