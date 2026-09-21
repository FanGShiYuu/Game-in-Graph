"""Schematic four-arm intersection demo."""

from .base import Route, Scenario


def build() -> Scenario:
    routes = {
        "n2_s2": Route.from_points("n2_s2", [(-2, 100), (-2, -100)]),
        "s2_n2": Route.from_points("s2_n2", [(2, -100), (2, 100)]),
        "e2_w2": Route.from_points("e2_w2", [(100, 2), (-100, 2)]),
        "w2_e2": Route.from_points("w2_e2", [(-100, -2), (100, -2)]),
    }
    return Scenario("intersection", routes)
