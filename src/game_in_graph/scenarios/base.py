"""Shared route geometry for compact scenario definitions."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class Route:
    route_id: str
    points: np.ndarray
    cumulative: np.ndarray

    @classmethod
    def from_points(cls, route_id: str, points: Iterable[tuple[float, float]]) -> "Route":
        array = np.asarray(list(points), dtype=float)
        if array.ndim != 2 or array.shape[0] < 2 or array.shape[1] != 2:
            raise ValueError(f"Route {route_id!r} requires at least two 2-D points.")
        segment_lengths = np.linalg.norm(np.diff(array, axis=0), axis=1)
        if np.any(segment_lengths <= 0):
            raise ValueError(f"Route {route_id!r} contains duplicate adjacent points.")
        cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        return cls(route_id=route_id, points=array, cumulative=cumulative)

    @property
    def length(self) -> float:
        return float(self.cumulative[-1])

    def pose_at(self, progress: float) -> tuple[float, float, float]:
        s = min(max(float(progress), 0.0), self.length)
        index = int(np.searchsorted(self.cumulative, s, side="right") - 1)
        index = min(index, len(self.points) - 2)
        start, end = self.points[index], self.points[index + 1]
        segment_length = self.cumulative[index + 1] - self.cumulative[index]
        ratio = (s - self.cumulative[index]) / segment_length
        position = start + ratio * (end - start)
        heading = math.atan2(end[1] - start[1], end[0] - start[0])
        return float(position[0]), float(position[1]), heading

    def sample(self, spacing: float = 0.5) -> tuple[np.ndarray, np.ndarray]:
        progress = np.linspace(0.0, self.length, max(2, int(self.length / spacing) + 1))
        points = np.asarray([self.pose_at(value)[:2] for value in progress])
        return progress, points


class Scenario:
    """A small route collection and cached geometric conflict lookup."""

    name: str

    def __init__(self, name: str, routes: dict[str, Route]) -> None:
        self.name = name
        self.routes = routes
        self._conflict_cache: dict[tuple[str, str], tuple[float, float] | None] = {}

    def route(self, route_id: str) -> Route:
        try:
            return self.routes[route_id]
        except KeyError as exc:
            raise ValueError(f"Unknown route {route_id!r} for scenario {self.name}.") from exc

    def conflict_progress(self, route_i: str, route_j: str) -> tuple[float, float] | None:
        """Return the first close point on two schematic centerlines."""

        if route_i == route_j:
            return None
        key = (route_i, route_j)
        if key in self._conflict_cache:
            return self._conflict_cache[key]
        progress_i, points_i = self.route(route_i).sample()
        progress_j, points_j = self.route(route_j).sample()
        distances = np.linalg.norm(points_i[:, None, :] - points_j[None, :, :], axis=2)
        candidates = np.argwhere(distances <= 0.75)
        result: tuple[float, float] | None = None
        if len(candidates):
            # Prefer the earliest encounter after both vehicles have entered the map.
            valid = [pair for pair in candidates if progress_i[pair[0]] > 10 and progress_j[pair[1]] > 10]
            if valid:
                pair = min(valid, key=lambda item: progress_i[item[0]] + progress_j[item[1]])
                result = (float(progress_i[pair[0]]), float(progress_j[pair[1]]))
        self._conflict_cache[key] = result
        reverse = None if result is None else (result[1], result[0])
        self._conflict_cache[(route_j, route_i)] = reverse
        return result
