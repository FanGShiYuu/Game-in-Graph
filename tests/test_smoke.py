from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import pytest

from game_in_graph import Simulation, load_config


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("scenario", ["intersection", "roundabout", "merging"])
def test_short_demo_completes_and_writes_portable_outputs(tmp_path: Path, scenario: str) -> None:
    config = load_config(ROOT / "configs" / f"{scenario}_demo.yaml")
    result = Simulation(config, output_dir=tmp_path / scenario).run()
    assert result.status == "completed"
    assert result.metrics["number_of_vehicles"] >= 3
    assert result.metrics["collision_count"] == 0
    assert result.metrics["completed_vehicles"] >= 1
    assert math.isfinite(result.metrics["average_delay_s"])
    for name in ("trajectories.csv", "edges.csv", "communities.csv", "metrics.json", "trajectory.png"):
        assert (result.output_dir / name).is_file()
    with (result.output_dir / "metrics.json").open("r", encoding="utf-8") as handle:
        metrics = json.load(handle)
    assert metrics["status"] == "completed"
    with (result.output_dir / "trajectories.csv").open("r", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert all(math.isfinite(float(row["x_m"])) for row in rows)
    serialized_paths = "\n".join(str(value) for row in rows for value in row.values())
    assert ":\\" not in serialized_paths
