#!/usr/bin/env python3
"""Frontier-based next-exploration-point selection from an occupancy grid,
ROS-independent.

Pure standard-library. No rclpy, no cv2, no numpy, no third-party dependencies.
The occupancy grid input is duck-typed to match the shape of a
`nav_msgs/OccupancyGrid` without importing `nav_msgs`, so this module stays
independently testable outside a ROS environment.
"""

from __future__ import annotations

import math
import sys
from collections import deque
from dataclasses import dataclass
from typing import Any, Optional

OCCUPIED_THRESHOLD = 65
FREE_THRESHOLD = 50

_NEIGHBOR_OFFSETS = ((-1, 0), (1, 0), (0, -1), (0, 1))


@dataclass(frozen=True)
class Frontier:
    x: float
    y: float
    size: int


def _cell_value(data: Any, width: int, row: int, col: int) -> Optional[int]:
    if row < 0 or col < 0:
        return None
    index = row * width + col
    if index >= len(data):
        return None
    return int(data[index])


def find_frontiers(grid: Any, *, min_cluster_cells: int = 6) -> list[Frontier]:
    """Cluster free cells adjacent to unknown space into exploration frontiers."""
    info = grid.info
    width = int(info.width)
    height = int(info.height)
    data = grid.data
    if width <= 0 or height <= 0 or len(data) == 0:
        return []

    resolution = float(info.resolution)
    origin_x = float(info.origin.position.x)
    origin_y = float(info.origin.position.y)

    frontier_cells: set[tuple[int, int]] = set()
    for row in range(height):
        for col in range(width):
            value = _cell_value(data, width, row, col)
            if value is None or value < 0 or value >= FREE_THRESHOLD:
                continue  # not a free cell (unknown, ambiguous, or occupied)

            has_unknown_neighbor = False
            near_occupied = False
            for delta_row, delta_col in _NEIGHBOR_OFFSETS:
                neighbor = _cell_value(data, width, row + delta_row, col + delta_col)
                if neighbor is None:
                    continue
                if neighbor == -1:
                    has_unknown_neighbor = True
                if neighbor >= OCCUPIED_THRESHOLD:
                    near_occupied = True
                    break
            if has_unknown_neighbor and not near_occupied:
                frontier_cells.add((row, col))

    visited: set[tuple[int, int]] = set()
    frontiers: list[Frontier] = []
    for cell in frontier_cells:
        if cell in visited:
            continue
        cluster: list[tuple[int, int]] = []
        queue: deque[tuple[int, int]] = deque([cell])
        visited.add(cell)
        while queue:
            current = queue.popleft()
            cluster.append(current)
            current_row, current_col = current
            for delta_row, delta_col in _NEIGHBOR_OFFSETS:
                neighbor = (current_row + delta_row, current_col + delta_col)
                if neighbor in frontier_cells and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        if len(cluster) < min_cluster_cells:
            continue

        mean_row = sum(item[0] for item in cluster) / len(cluster)
        mean_col = sum(item[1] for item in cluster) / len(cluster)
        world_x = origin_x + (mean_col + 0.5) * resolution
        world_y = origin_y + (mean_row + 0.5) * resolution
        frontiers.append(Frontier(world_x, world_y, len(cluster)))

    return frontiers


def pick_frontier(
    frontiers: list[Frontier],
    robot_x: float,
    robot_y: float,
    visited: list[tuple[float, float]],
    *,
    min_distance_m: float = 0.5,
    visited_radius_m: float = 0.6,
) -> Optional[Frontier]:
    """Pick the best unvisited, sufficiently-distant frontier to explore next."""
    best: Optional[Frontier] = None
    best_score = -math.inf
    for frontier in frontiers:
        distance_to_robot = math.hypot(frontier.x - robot_x, frontier.y - robot_y)
        if distance_to_robot < min_distance_m:
            continue
        if any(
            math.hypot(frontier.x - vx, frontier.y - vy) <= visited_radius_m
            for vx, vy in visited
        ):
            continue
        score = frontier.size / (1.0 + distance_to_robot)
        if score > best_score:
            best_score = score
            best = frontier
    return best


class _FakeInfo:
    def __init__(self, width: int, height: int, resolution: float, origin_x: float, origin_y: float) -> None:
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = _FakeOrigin(origin_x, origin_y)


class _FakeOrigin:
    def __init__(self, x: float, y: float) -> None:
        self.position = _FakePosition(x, y)


class _FakePosition:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y


class _FakeGrid:
    def __init__(self, info: _FakeInfo, data: list[int]) -> None:
        self.info = info
        self.data = data


def _build_synthetic_grid() -> _FakeGrid:
    # 10x10 grid, all unknown, with a 4x4 free block carved in the middle
    # (rows 3-6, cols 3-6) and a wall of occupied cells along column 9.
    width, height = 10, 10
    data = [-1] * (width * height)

    def set_cell(row: int, col: int, value: int) -> None:
        data[row * width + col] = value

    for row in range(3, 7):
        for col in range(3, 7):
            set_cell(row, col, 0)  # free

    for row in range(height):
        set_cell(row, 9, 100)  # occupied wall

    info = _FakeInfo(width, height, 0.1, 0.0, 0.0)
    return _FakeGrid(info, data)


def _smoke_test() -> None:
    grid = _build_synthetic_grid()
    frontiers = find_frontiers(grid, min_cluster_cells=3)
    assert frontiers, "expected at least one frontier cluster near the free/unknown boundary"

    wall_x = 0.0 + (9 + 0.5) * 0.1  # world x of the wall column
    for frontier in frontiers:
        assert (
            abs(frontier.x - wall_x) > 1e-9
        ), "no frontier should be adjacent to the occupied wall"

    robot_x, robot_y = 0.0, 0.0
    best = pick_frontier(frontiers, robot_x, robot_y, visited=[])
    assert best is not None, "expected pick_frontier to return a candidate"

    visited = [(best.x, best.y)]
    excluded = pick_frontier(
        frontiers, robot_x, robot_y, visited=visited, visited_radius_m=0.6
    )
    assert excluded is None or excluded.x != best.x or excluded.y != best.y, (
        "pick_frontier should exclude a frontier inside the visited circle"
    )

    empty_grid = _FakeGrid(_FakeInfo(0, 0, 0.1, 0.0, 0.0), [])
    assert find_frontiers(empty_grid) == [], "empty grid should yield no frontiers"

    print("OK")


if __name__ == "__main__":
    _smoke_test()
    sys.exit(0)
