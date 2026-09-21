"""Spatiotemporal conflict topological graph construction."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .scenarios.base import Scenario
from .vehicle import VehicleState


@dataclass(frozen=True)
class ConflictEdge:
    source: int
    target: int
    edge_type: str
    weight: float
    delta_ttcp: float | None
    distance_to_conflict_source: float | None
    distance_to_conflict_target: float | None
    critical: bool


@dataclass(frozen=True)
class InteractionGraph:
    node_ids: tuple[int, ...]
    free_agent_ids: tuple[int, ...]
    edges: tuple[ConflictEdge, ...]

    def edge_pairs(self) -> set[tuple[int, int]]:
        return {(min(edge.source, edge.target), max(edge.source, edge.target)) for edge in self.edges}


class GraphBuilder:
    """Build source-aligned unsmoothed TTCP and following edges."""

    def __init__(self, scenario: Scenario, graph_config: dict, planning_horizon: float) -> None:
        self.scenario = scenario
        self.free_agent_distance = float(graph_config["free_agent_distance"])
        self.ttcp_threshold = float(graph_config["ttcp_threshold"])
        self.critical_ttcp = float(graph_config["critical_ttcp"])
        self.critical_weight = float(graph_config["critical_weight"])
        self.planning_horizon = float(planning_horizon)

    def build(self, vehicles: list[VehicleState]) -> InteractionGraph:
        ordered = sorted(vehicles, key=lambda item: item.vehicle_id)
        free = {
            vehicle.vehicle_id
            for vehicle in ordered
            if self.scenario.route(vehicle.route_id).length - vehicle.progress <= self.free_agent_distance
        }
        edges: list[ConflictEdge] = []
        for position, vehicle_i in enumerate(ordered):
            for vehicle_j in ordered[:position]:
                if vehicle_i.vehicle_id in free or vehicle_j.vehicle_id in free:
                    continue
                edge = self._edge(vehicle_i, vehicle_j)
                if edge is not None:
                    edges.append(edge)
        return InteractionGraph(
            node_ids=tuple(vehicle.vehicle_id for vehicle in ordered),
            free_agent_ids=tuple(sorted(free)),
            edges=tuple(edges),
        )

    def _edge(self, vehicle_i: VehicleState, vehicle_j: VehicleState) -> ConflictEdge | None:
        if vehicle_i.route_id == vehicle_j.route_id:
            leader, follower = sorted((vehicle_i, vehicle_j), key=lambda item: item.progress, reverse=True)
            gap = leader.progress - follower.progress
            reachable = follower.speed * self.planning_horizon
            if gap <= reachable:
                return ConflictEdge(
                    source=follower.vehicle_id,
                    target=leader.vehicle_id,
                    edge_type="following",
                    weight=self.critical_weight,
                    delta_ttcp=None,
                    distance_to_conflict_source=gap,
                    distance_to_conflict_target=0.0,
                    critical=True,
                )
            return None

        conflict = self.scenario.conflict_progress(vehicle_i.route_id, vehicle_j.route_id)
        if conflict is None:
            return None
        distance_i = conflict[0] - vehicle_i.progress
        distance_j = conflict[1] - vehicle_j.progress
        if distance_i < 0 or distance_j < 0:
            return None
        # Stationary vehicles remain represented with an infinite arrival time.
        ttcp_i = distance_i / vehicle_i.speed if vehicle_i.speed > 1e-6 else math.inf
        ttcp_j = distance_j / vehicle_j.speed if vehicle_j.speed > 1e-6 else math.inf
        delta = abs(ttcp_i - ttcp_j)
        if not math.isfinite(delta):
            weight = 0.0
            critical = False
        else:
            critical = delta < self.critical_ttcp
            weight = self.critical_weight if critical else math.exp(self.ttcp_threshold - delta)
        return ConflictEdge(
            source=vehicle_i.vehicle_id,
            target=vehicle_j.vehicle_id,
            edge_type="crossing",
            weight=weight,
            delta_ttcp=delta,
            distance_to_conflict_source=distance_i,
            distance_to_conflict_target=distance_j,
            critical=critical,
        )
