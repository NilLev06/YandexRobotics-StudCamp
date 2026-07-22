#!/usr/bin/env python3
"""Pure-Python frontier detection and goal selection for occupancy grids.

The module has no ROS dependencies, which makes the exploration policy easy to
unit-test on a development machine before it is deployed to the rover.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, Sequence

Cell = tuple[int, int]
Pose2D = tuple[float, float, float]


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def valid(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def index(self, x: int, y: int) -> int:
        return y * self.width + x

    def grid_to_world(self, cell: Cell) -> tuple[float, float]:
        gx = (cell[0] + 0.5) * self.resolution
        gy = (cell[1] + 0.5) * self.resolution
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        return (
            self.origin_x + c * gx - s * gy,
            self.origin_y + s * gx + c * gy,
        )

    def world_to_grid(self, x: float, y: float) -> Cell:
        dx = x - self.origin_x
        dy = y - self.origin_y
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        gx = c * dx + s * dy
        gy = -s * dx + c * dy
        return (math.floor(gx / self.resolution), math.floor(gy / self.resolution))


@dataclass(frozen=True)
class FrontierGoal:
    cell: Cell
    world_x: float
    world_y: float
    yaw: float
    path_distance_m: float
    frontier_size_m: float
    score: float
    cluster_cells: tuple[Cell, ...]


_NEIGHBORS_4: tuple[Cell, ...] = ((1, 0), (-1, 0), (0, 1), (0, -1))
_NEIGHBORS_8: tuple[Cell, ...] = (
    (1, 0), (-1, 0), (0, 1), (0, -1),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
)


def _classify(
    data: Sequence[int],
    free_threshold: int,
    occupied_threshold: int,
) -> tuple[list[bool], list[bool], list[bool]]:
    free = [False] * len(data)
    occupied = [False] * len(data)
    unknown = [False] * len(data)
    for i, value in enumerate(data):
        if value < 0:
            unknown[i] = True
        elif value <= free_threshold:
            free[i] = True
        elif value >= occupied_threshold:
            occupied[i] = True
    return free, occupied, unknown


def inflate_obstacles(
    spec: GridSpec,
    occupied: Sequence[bool],
    radius_m: float,
) -> list[bool]:
    """Return a boolean mask with occupied cells dilated by ``radius_m``."""
    unsafe = list(occupied)
    radius_cells = max(0, math.ceil(radius_m / spec.resolution))
    if radius_cells == 0:
        return unsafe

    offsets: list[Cell] = []
    r2 = radius_cells * radius_cells
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= r2:
                offsets.append((dx, dy))

    occupied_cells = [
        (i % spec.width, i // spec.width)
        for i, value in enumerate(occupied)
        if value
    ]
    for ox, oy in occupied_cells:
        for dx, dy in offsets:
            x, y = ox + dx, oy + dy
            if spec.valid(x, y):
                unsafe[spec.index(x, y)] = True
    return unsafe


def nearest_traversable_seed(
    spec: GridSpec,
    preferred: Cell,
    traversable: Sequence[bool],
) -> Cell | None:
    """Find the nearest traversable cell to a possibly invalid robot cell."""
    px = min(max(preferred[0], 0), spec.width - 1)
    py = min(max(preferred[1], 0), spec.height - 1)
    if traversable[spec.index(px, py)]:
        return (px, py)

    queue: deque[Cell] = deque([(px, py)])
    seen = {(px, py)}
    while queue:
        x, y = queue.popleft()
        for dx, dy in _NEIGHBORS_8:
            nx, ny = x + dx, y + dy
            if not spec.valid(nx, ny) or (nx, ny) in seen:
                continue
            if traversable[spec.index(nx, ny)]:
                return (nx, ny)
            seen.add((nx, ny))
            queue.append((nx, ny))
    return None


def reachable_distances(
    spec: GridSpec,
    seed: Cell,
    traversable: Sequence[bool],
) -> list[int]:
    """Compute shortest 4-connected grid distances from ``seed``."""
    distance = [-1] * (spec.width * spec.height)
    start_idx = spec.index(*seed)
    if not traversable[start_idx]:
        return distance
    distance[start_idx] = 0
    queue: deque[Cell] = deque([seed])
    while queue:
        x, y = queue.popleft()
        next_distance = distance[spec.index(x, y)] + 1
        for dx, dy in _NEIGHBORS_4:
            nx, ny = x + dx, y + dy
            if not spec.valid(nx, ny):
                continue
            idx = spec.index(nx, ny)
            if distance[idx] >= 0 or not traversable[idx]:
                continue
            distance[idx] = next_distance
            queue.append((nx, ny))
    return distance


def _is_frontier(
    spec: GridSpec,
    cell: Cell,
    free: Sequence[bool],
    unknown: Sequence[bool],
    distance: Sequence[int],
) -> bool:
    idx = spec.index(*cell)
    if not free[idx] or distance[idx] < 0:
        return False
    x, y = cell
    for dx, dy in _NEIGHBORS_8:
        nx, ny = x + dx, y + dy
        if spec.valid(nx, ny) and unknown[spec.index(nx, ny)]:
            return True
    return False


def cluster_frontiers(
    spec: GridSpec,
    free: Sequence[bool],
    unknown: Sequence[bool],
    distance: Sequence[int],
) -> list[list[Cell]]:
    frontier_mask = [False] * (spec.width * spec.height)
    for y in range(spec.height):
        for x in range(spec.width):
            if _is_frontier(spec, (x, y), free, unknown, distance):
                frontier_mask[spec.index(x, y)] = True

    clusters: list[list[Cell]] = []
    visited: set[Cell] = set()
    for y in range(spec.height):
        for x in range(spec.width):
            start = (x, y)
            if start in visited or not frontier_mask[spec.index(x, y)]:
                continue
            cluster: list[Cell] = []
            queue: deque[Cell] = deque([start])
            visited.add(start)
            while queue:
                cell = queue.popleft()
                cluster.append(cell)
                cx, cy = cell
                for dx, dy in _NEIGHBORS_8:
                    nxt = (cx + dx, cy + dy)
                    if (
                        spec.valid(*nxt)
                        and nxt not in visited
                        and frontier_mask[spec.index(*nxt)]
                    ):
                        visited.add(nxt)
                        queue.append(nxt)
            clusters.append(cluster)
    return clusters


def _angle_to_unknown(
    spec: GridSpec,
    cluster: Sequence[Cell],
    unknown: Sequence[bool],
    fallback_yaw: float,
) -> float:
    vectors: list[tuple[float, float]] = []
    for x, y in cluster:
        for dx, dy in _NEIGHBORS_8:
            nx, ny = x + dx, y + dy
            if spec.valid(nx, ny) and unknown[spec.index(nx, ny)]:
                vectors.append((float(dx), float(dy)))
    if not vectors:
        return fallback_yaw
    vx = sum(v[0] for v in vectors)
    vy = sum(v[1] for v in vectors)
    return math.atan2(vy, vx) + spec.origin_yaw


def _is_blacklisted(
    x: float,
    y: float,
    blacklist: Iterable[tuple[float, float]],
    radius_m: float,
) -> bool:
    r2 = radius_m * radius_m
    return any((x - bx) ** 2 + (y - by) ** 2 <= r2 for bx, by in blacklist)


def select_frontier_goals(
    *,
    spec: GridSpec,
    data: Sequence[int],
    robot_pose: Pose2D,
    inflation_radius_m: float,
    min_frontier_size_m: float,
    min_goal_distance_m: float,
    free_threshold: int = 20,
    occupied_threshold: int = 65,
    gain_weight: float = 1.0,
    blacklist: Iterable[tuple[float, float]] = (),
    blacklist_radius_m: float = 0.6,
) -> list[FrontierGoal]:
    """Return reachable frontier goals sorted by cost (best first)."""
    expected = spec.width * spec.height
    if len(data) != expected or expected == 0 or spec.resolution <= 0.0:
        return []

    free, occupied, unknown = _classify(
        data, free_threshold, occupied_threshold
    )
    unsafe = inflate_obstacles(spec, occupied, inflation_radius_m)
    traversable = [f and not u for f, u in zip(free, unsafe)]

    robot_cell = spec.world_to_grid(robot_pose[0], robot_pose[1])
    seed = nearest_traversable_seed(spec, robot_cell, traversable)
    if seed is None:
        return []
    distance = reachable_distances(spec, seed, traversable)
    clusters = cluster_frontiers(spec, free, unknown, distance)

    minimum_cells = max(1, math.ceil(min_frontier_size_m / spec.resolution))
    goals: list[FrontierGoal] = []
    for cluster in clusters:
        if len(cluster) < minimum_cells:
            continue

        # Pick the reachable cell closest to the cluster centroid. This keeps the
        # goal on known free space instead of placing it inside the unknown map.
        centroid_x = sum(c[0] for c in cluster) / len(cluster)
        centroid_y = sum(c[1] for c in cluster) / len(cluster)
        candidates = [c for c in cluster if distance[spec.index(*c)] >= 0]
        if not candidates:
            continue
        cell = min(
            candidates,
            key=lambda c: (
                (c[0] - centroid_x) ** 2 + (c[1] - centroid_y) ** 2,
                distance[spec.index(*c)],
            ),
        )
        path_distance_m = distance[spec.index(*cell)] * spec.resolution
        if path_distance_m < min_goal_distance_m:
            continue
        world_x, world_y = spec.grid_to_world(cell)
        if _is_blacklisted(
            world_x, world_y, blacklist, blacklist_radius_m
        ):
            continue

        frontier_size_m = len(cluster) * spec.resolution
        score = path_distance_m - gain_weight * math.sqrt(len(cluster)) * spec.resolution
        yaw = _angle_to_unknown(spec, cluster, unknown, robot_pose[2])
        goals.append(
            FrontierGoal(
                cell=cell,
                world_x=world_x,
                world_y=world_y,
                yaw=yaw,
                path_distance_m=path_distance_m,
                frontier_size_m=frontier_size_m,
                score=score,
                cluster_cells=tuple(cluster),
            )
        )

    goals.sort(key=lambda goal: (goal.score, goal.path_distance_m))
    return goals
