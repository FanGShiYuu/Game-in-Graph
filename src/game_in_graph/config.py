"""Configuration loading and validation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class DemoConfig:
    """Validated configuration for a short deterministic simulation."""

    simulation: dict[str, Any]
    vehicle: dict[str, float]
    graph: dict[str, float | int]
    solver: dict[str, Any]
    vehicles: list[dict[str, Any]]
    source_path: Path

    @property
    def scenario(self) -> str:
        return str(self.simulation["scenario"])

    @property
    def output_dir(self) -> Path:
        return Path(str(self.simulation["output_dir"]))


def load_config(path: str | Path) -> DemoConfig:
    """Load a YAML demo configuration and reject incomplete inputs."""

    source = Path(path).resolve()
    with source.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError("The configuration root must be a mapping.")
    required = {"simulation", "vehicle", "graph", "solver", "vehicles"}
    missing = required.difference(data)
    if missing:
        raise ValueError(f"Missing configuration sections: {sorted(missing)}")
    scenario = data["simulation"].get("scenario")
    if scenario not in {"intersection", "roundabout", "merging"}:
        raise ValueError(f"Unsupported scenario: {scenario!r}")
    ids = [int(item["id"]) for item in data["vehicles"]]
    if len(ids) != len(set(ids)):
        raise ValueError("Vehicle IDs must be unique.")
    if float(data["simulation"]["dt"]) <= 0:
        raise ValueError("The control period must be positive.")
    return DemoConfig(
        simulation=dict(data["simulation"]),
        vehicle=dict(data["vehicle"]),
        graph=dict(data["graph"]),
        solver=dict(data["solver"]),
        vehicles=[dict(item) for item in data["vehicles"]],
        source_path=source,
    )
