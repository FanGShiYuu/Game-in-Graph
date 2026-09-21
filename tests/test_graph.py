from __future__ import annotations

import math
from pathlib import Path

from game_in_graph.config import load_config
from game_in_graph.interaction_graph import GraphBuilder
from game_in_graph.scenarios import load_scenario
from game_in_graph.vehicle import VehicleState


ROOT = Path(__file__).resolve().parents[1]


def test_intersection_edge_uses_verified_ttcp_weight() -> None:
    config = load_config(ROOT / "configs" / "intersection_demo.yaml")
    scenario = load_scenario("intersection")
    builder = GraphBuilder(scenario, config.graph, config.simulation["planning_horizon"])
    vehicles = [
        VehicleState(1, "cav", "n2_s2", 60.0, 8.0),
        VehicleState(2, "cav", "e2_w2", 50.0, 8.0),
    ]
    graph = builder.build(vehicles)
    assert len(graph.edges) == 1
    edge = graph.edges[0]
    assert edge.edge_type == "crossing"
    assert edge.delta_ttcp is not None
    expected = math.exp(float(config.graph["ttcp_threshold"]) - edge.delta_ttcp)
    assert math.isclose(edge.weight, expected, rel_tol=1e-12)


def test_critical_edge_uses_forced_weight() -> None:
    config = load_config(ROOT / "configs" / "intersection_demo.yaml")
    scenario = load_scenario("intersection")
    builder = GraphBuilder(scenario, config.graph, config.simulation["planning_horizon"])
    vehicles = [
        VehicleState(1, "cav", "n2_s2", 60.0, 8.0),
        VehicleState(2, "cav", "e2_w2", 64.0, 8.0),
    ]
    edge = builder.build(vehicles).edges[0]
    assert edge.critical
    assert edge.weight == float(config.graph["critical_weight"])
