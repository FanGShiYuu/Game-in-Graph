#!/usr/bin/env python3
"""Plot an existing trajectory CSV without rerunning a simulation."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory_csv", type=Path)
    parser.add_argument("--output", type=Path, default=Path("trajectory.png"))
    args = parser.parse_args()
    with args.trajectory_csv.open("r", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    by_vehicle: dict[int, list[dict[str, str]]] = {}
    for row in rows:
        by_vehicle.setdefault(int(row["vehicle_id"]), []).append(row)
    figure, axis = plt.subplots(figsize=(6, 6))
    for vehicle_id, records in sorted(by_vehicle.items()):
        axis.plot(
            [float(row["x_m"]) for row in records],
            [float(row["y_m"]) for row in records],
            label=f"Vehicle {vehicle_id}",
        )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title("Game in Graph trajectory output")
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=150)
    plt.close(figure)
    print(f"Plot written to: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
