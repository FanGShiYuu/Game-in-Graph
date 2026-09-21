from pathlib import Path

from game_in_graph.config import load_config
from game_in_graph.solver import EnumerationSolver, build_solver


def test_default_backend_is_enumeration() -> None:
    root = Path(__file__).resolve().parents[1]
    config = load_config(root / "configs" / "intersection_demo.yaml")
    solver = build_solver(
        config.solver,
        config.vehicle,
        float(config.simulation["planning_horizon"]),
    )
    assert isinstance(solver, EnumerationSolver)
    assert solver.backend == "enumeration"
