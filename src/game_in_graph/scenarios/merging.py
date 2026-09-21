"""Schematic mainline/ramp merging-area demo."""

from .base import Route, Scenario


def build() -> Scenario:
    routes = {
        "s_s": Route.from_points("s_s", [(-120, 0), (120, 0)]),
        "m_s": Route.from_points("m_s", [(-110, -35), (-25, -12), (0, 0), (120, 0)]),
    }
    return Scenario("merging", routes)
