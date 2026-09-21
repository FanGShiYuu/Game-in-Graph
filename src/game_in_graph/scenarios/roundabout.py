"""Schematic single-lane roundabout demo."""

from __future__ import annotations

import math

from .base import Route, Scenario


ENTRY_ANGLE = {"e": 0.0, "n": math.pi / 2, "w": math.pi, "s": 3 * math.pi / 2}


def _route(route_id: str, entrance: str, exit_: str) -> Route:
    radius = 20.0
    outer = 100.0
    start_angle = ENTRY_ANGLE[entrance]
    end_angle = ENTRY_ANGLE[exit_]
    while end_angle <= start_angle:
        end_angle += 2 * math.pi
    points = [
        (outer * math.cos(start_angle), outer * math.sin(start_angle)),
        (radius * math.cos(start_angle), radius * math.sin(start_angle)),
    ]
    steps = max(3, int((end_angle - start_angle) / (math.pi / 24)))
    for index in range(1, steps + 1):
        angle = start_angle + (end_angle - start_angle) * index / steps
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    points.append((outer * math.cos(end_angle), outer * math.sin(end_angle)))
    return Route.from_points(route_id, points)


def build() -> Scenario:
    routes = {
        "s_e": _route("s_e", "s", "e"),
        "w_n": _route("w_n", "w", "n"),
        "e_w": _route("e_w", "e", "w"),
        "n_s": _route("n_s", "n", "s"),
    }
    return Scenario("roundabout", routes)
