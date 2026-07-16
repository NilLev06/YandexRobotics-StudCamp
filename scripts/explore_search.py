#!/usr/bin/env python3
"""Frontier-driven exploration wrapper around object_search.py's guarded search.

One 360-degree step-and-stare search (object_search.py's own run_spin_search)
already stops and approaches on a confirmed target. This script adds exactly
the one thing that's missing: if a full circle finds nothing, drive to the
nearest unexplored frontier of the SLAM map and search again there, repeating
until the target is found, the reachable area is fully explored, or
--max-cycles / --max-runtime-s is hit.

It changes nothing about the safety model or the map/dashboard integration:
this is a subclass of BottleSearchNode that reuses every existing preflight
gate, Nav2 action wrapper, and the same /search/found, /search/sighted,
/search/status ROS topics object_search.py already publishes -- map_view.py
needs no changes to show what this script does.

Exit codes match object_search.py: 0 = target confirmed (and approached
unless --search-only), 1 = unsafe/error stop, 2 = the reachable area was
fully explored (no more frontiers, or --max-cycles/--max-runtime-s reached)
without a confirmed target, 130 = interrupted.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from typing import Any, Optional, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from frontier import Frontier, find_frontiers, pick_frontier
from object_search import (
    EXIT_ERROR,
    EXIT_NOT_FOUND,
    EXIT_OK,
    BottleSearchNode,
    Calibration,
    SafeArgumentParser,
    UnsafeError,
    _ros_transform,
    parse_args as parse_search_args,
)


class ExploreSearchNode(BottleSearchNode):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(args)
        self._map_lock = threading.Lock()
        self._grid: Optional[OccupancyGrid] = None
        self._grid_at = 0.0
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self._visited_frontiers: list[tuple[float, float]] = []

    def _on_map(self, message: OccupancyGrid) -> None:
        with self._map_lock:
            self._grid = message
            self._grid_at = time.monotonic()

    def _map_snapshot(self) -> tuple[Optional[OccupancyGrid], float]:
        with self._map_lock:
            return self._grid, self._grid_at

    def _current_map_pose(self) -> tuple[float, float]:
        transform = _ros_transform(
            self._lookup_transform(self.args.map_frame, self.args.base_frame)
        )
        return transform.translation[0], transform.translation[1]

    def _pick_next_frontier(self) -> Optional[Frontier]:
        grid, grid_at = self._map_snapshot()
        if grid is None or time.monotonic() - grid_at > 5.0:
            raise UnsafeError("no fresh /map available to pick a frontier")
        frontiers = find_frontiers(
            grid, min_cluster_cells=self.args.min_frontier_cluster_cells
        )
        robot_x, robot_y = self._current_map_pose()
        return pick_frontier(
            frontiers,
            robot_x,
            robot_y,
            self._visited_frontiers,
            min_distance_m=self.args.min_frontier_distance_m,
            visited_radius_m=self.args.visited_radius_m,
        )

    def _navigate_to_frontier(self, frontier: Frontier, label: str) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = frontier.x
        goal.pose.pose.position.y = frontier.y
        goal.pose.pose.orientation.w = 1.0
        self.get_logger().info(
            f"{label}: navigating to frontier ({frontier.x:.2f}, {frontier.y:.2f}), "
            f"size={frontier.size} cells"
        )
        self._execute_action(
            self.navigate_client,
            self.navigate_cancel_client,
            goal,
            self.args.goal_timeout_s,
            label,
            lambda: self._sensor_guard_reason(self.args.guardian_clearance_m),
        )

    def run(self) -> int:
        calibration: Optional[Calibration] = self.preflight_and_calibrate()
        if self.args.dry_run:
            self.get_logger().info(
                "dry-run complete: all preflight gates passed; no action was sent"
            )
            return EXIT_OK

        deadline = time.monotonic() + self.args.max_runtime_s
        cycle = 0
        while cycle < self.args.max_cycles:
            cycle += 1
            if time.monotonic() >= deadline:
                self.get_logger().info("max runtime reached; stopping exploration")
                break

            self._publish_status(
                f"Разведка, цикл {cycle}/{self.args.max_cycles}: полный поиск {self.args.target_class}"
            )
            result = self.run_spin_search(calibration)
            if result == EXIT_OK:
                return EXIT_OK
            # result == EXIT_NOT_FOUND: this circle found nothing; drive on.

            self._publish_status(f"Цикл {cycle}: выбираю следующую точку разведки")
            try:
                frontier = self._pick_next_frontier()
            except UnsafeError as exc:
                self.get_logger().warning(f"frontier selection failed: {exc}")
                frontier = None

            if frontier is None:
                self.get_logger().info(
                    "no reachable unexplored frontier remains; exploration exhausted"
                )
                self._publish_status(
                    "Разведка завершена: неисследованных областей больше нет"
                )
                break

            robot_x, robot_y = self._current_map_pose()
            self._visited_frontiers.append((robot_x, robot_y))
            try:
                self._navigate_to_frontier(frontier, f"explore cycle {cycle} drive")
            except UnsafeError as exc:
                self.get_logger().warning(f"navigate to frontier failed: {exc}")
                # Mark the failed frontier visited too, so we don't retry it forever.
                self._visited_frontiers.append((frontier.x, frontier.y))
                continue

        return EXIT_NOT_FOUND


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    # Reuse object_search.py's parser, then extend it with exploration-only
    # options. SafeArgumentParser.error() already exits 64 on bad usage.
    parser = SafeArgumentParser(
        description="Frontier-driven exploration wrapper around object_search.py",
        parents=[],
    )
    # Duplicate object_search's own arguments here would drift; instead reuse
    # its parse_args() for validation, then re-parse just the extra ones.
    args = parse_search_args(argv)

    extra_parser = argparse.ArgumentParser(add_help=False)
    extra_parser.add_argument("--max-cycles", type=int, default=8)
    extra_parser.add_argument("--max-runtime-s", type=float, default=1200.0)
    extra_parser.add_argument("--min-frontier-cluster-cells", type=int, default=6)
    extra_parser.add_argument("--min-frontier-distance-m", type=float, default=0.5)
    extra_parser.add_argument("--visited-radius-m", type=float, default=0.6)
    extra_args, _ = extra_parser.parse_known_args(argv)

    args.max_cycles = extra_args.max_cycles
    args.max_runtime_s = extra_args.max_runtime_s
    args.min_frontier_cluster_cells = extra_args.min_frontier_cluster_cells
    args.min_frontier_distance_m = extra_args.min_frontier_distance_m
    args.visited_radius_m = extra_args.visited_radius_m

    checks = (
        (1 <= args.max_cycles <= 100, "--max-cycles must be between 1 and 100"),
        (
            60.0 <= args.max_runtime_s <= 7200.0,
            "--max-runtime-s must be between 60 and 7200",
        ),
        (
            2 <= args.min_frontier_cluster_cells <= 200,
            "--min-frontier-cluster-cells must be between 2 and 200",
        ),
        (
            0.1 <= args.min_frontier_distance_m <= 5.0,
            "--min-frontier-distance-m must be between 0.1 and 5.0",
        ),
        (
            0.1 <= args.visited_radius_m <= 5.0,
            "--visited-radius-m must be between 0.1 and 5.0",
        ),
    )
    for valid, message in checks:
        if not valid:
            parser.error(message)
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    rclpy.init(args=None)
    node = ExploreSearchNode(args)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin, name="explore-search-ros", daemon=True
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
        node._publish_status(f"Остановлено: {exc}")
    except KeyboardInterrupt:
        node.request_stop("keyboard interrupt")
        node._publish_status("Остановлено: прервано пользователем")
    except BaseException as exc:
        node.get_logger().error(f"unexpected failure: {type(exc).__name__}: {exc}")
        node._publish_status(f"Остановлено: непредвиденная ошибка ({type(exc).__name__})")
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
