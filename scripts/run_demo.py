#!/usr/bin/env python3
"""Run one compact Game in Graph scenario."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from game_in_graph import Simulation, load_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("intersection", "roundabout", "merging"), required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_config(args.config)
    if config.scenario != args.scenario:
        raise ValueError(
            f"Scenario argument {args.scenario!r} does not match config value {config.scenario!r}."
        )
    result = Simulation(config, output_dir=args.output).run()
    metrics = result.metrics
    print(f"Simulation status: {result.status}")
    print(f"Number of vehicles: {metrics['number_of_vehicles']}")
    print(f"Number of SCTG edges: {metrics['number_of_sctg_edges']}")
    print(f"Detected communities: {metrics['detected_communities']}")
    print(f"Minimum delta-TTCP: {metrics['minimum_delta_ttcp_s']}")
    print(f"Completion count: {metrics['completed_vehicles']}")
    print(f"Average delay: {metrics['average_delay_s']:.4f} s")
    print(f"Trajectory CSV: {result.output_dir / 'trajectories.csv'}")
    print(f"Trajectory plot: {result.output_dir / 'trajectory.png'}")
    return 0 if result.status == "completed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
