"""Scenario registry."""

from .base import Scenario


def load_scenario(name: str) -> Scenario:
    if name == "intersection":
        from .intersection import build
    elif name == "roundabout":
        from .roundabout import build
    elif name == "merging":
        from .merging import build
    else:
        raise ValueError(f"Unsupported scenario: {name!r}")
    return build()


__all__ = ["Scenario", "load_scenario"]
