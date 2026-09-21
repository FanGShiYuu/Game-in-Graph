"""Small safety and efficiency metrics for demo validation."""

from __future__ import annotations

import math

from .scenarios.base import Scenario
from .vehicle import VehicleState


def oriented_rectangle(state: VehicleState, scenario: Scenario, length: float, width: float):
    x, y, heading = scenario.route(state.route_id).pose_at(state.progress)
    cosine, sine = math.cos(heading), math.sin(heading)
    forward = (cosine * length / 2, sine * length / 2)
    lateral = (-sine * width / 2, cosine * width / 2)
    return [
        (x + forward[0] + lateral[0], y + forward[1] + lateral[1]),
        (x + forward[0] - lateral[0], y + forward[1] - lateral[1]),
        (x - forward[0] - lateral[0], y - forward[1] - lateral[1]),
        (x - forward[0] + lateral[0], y - forward[1] + lateral[1]),
    ]


def _project(polygon, axis):
    values = [point[0] * axis[0] + point[1] * axis[1] for point in polygon]
    return min(values), max(values)


def rectangles_overlap(first, second) -> bool:
    for polygon in (first, second):
        for index in range(len(polygon)):
            start, end = polygon[index], polygon[(index + 1) % len(polygon)]
            edge = (end[0] - start[0], end[1] - start[1])
            axis = (-edge[1], edge[0])
            norm = math.hypot(*axis)
            axis = (axis[0] / norm, axis[1] / norm)
            min_a, max_a = _project(first, axis)
            min_b, max_b = _project(second, axis)
            if max_a < min_b or max_b < min_a:
                return False
    return True


def collision_pairs(vehicles: list[VehicleState], scenario: Scenario, length: float, width: float):
    collisions = []
    for index, vehicle_i in enumerate(vehicles):
        polygon_i = oriented_rectangle(vehicle_i, scenario, length, width)
        for vehicle_j in vehicles[:index]:
            polygon_j = oriented_rectangle(vehicle_j, scenario, length, width)
            if rectangles_overlap(polygon_i, polygon_j):
                collisions.append(tuple(sorted((vehicle_i.vehicle_id, vehicle_j.vehicle_id))))
    return collisions
