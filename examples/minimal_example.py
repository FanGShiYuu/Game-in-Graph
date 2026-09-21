"""Minimal Python API example."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from game_in_graph import Simulation, load_config


config = load_config(ROOT / "configs" / "intersection_demo.yaml")
result = Simulation(config, output_dir=ROOT / "results" / "minimal").run()
print(result.metrics)
