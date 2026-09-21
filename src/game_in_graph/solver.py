"""Bounded cooperative decision backend for lightweight method inspection."""

from __future__ import annotations

from itertools import product
import math

from .interaction_graph import ConflictEdge, InteractionGraph
from .vehicle import VehicleState


class InspectionSolver:
    """Enumerate a small action grid under conflict-order safety constraints.

    This is the public preview backend, not the unreleased full Gurobi model.
    Its purpose is to make the graph-to-subgroup-to-action flow executable.
    """

    def __init__(self, config: dict, vehicle_config: dict, planning_horizon: float) -> None:
        self.candidates = tuple(float(item) for item in config["acceleration_candidates"])
        self.efficiency_weight = float(config["efficiency_weight"])
        self.safety_weight = float(config["safety_weight"])
        self.min_ttcp_gap = float(config["min_predicted_ttcp_gap"])
        self.min_acceleration = float(vehicle_config["min_acceleration"])
        self.max_acceleration = float(vehicle_config["max_acceleration"])
        self.speed_limit = float(vehicle_config["speed_limit"])
        self.vehicle_length = float(vehicle_config["length"])
        self.stop_gap = float(vehicle_config["min_stop_gap"])
        self.horizon = float(planning_horizon)

    def solve(
        self,
        vehicles: list[VehicleState],
        graph: InteractionGraph,
        communities: list[list[int]],
    ) -> dict[int, float]:
        by_id = {vehicle.vehicle_id: vehicle for vehicle in vehicles}
        actions = {vehicle.vehicle_id: self.max_acceleration for vehicle in vehicles}
        free_ids = set(graph.free_agent_ids)
        for community in communities:
            controlled = [node for node in community if node in by_id and node not in free_ids]
            if not controlled:
                continue
            local_edges = [
                edge for edge in graph.edges if edge.source in controlled or edge.target in controlled
            ]
            best = self._solve_group(controlled, by_id, local_edges)
            actions.update(best)
        return actions

    def _solve_group(
        self,
        vehicle_ids: list[int],
        by_id: dict[int, VehicleState],
        edges: list[ConflictEdge],
    ) -> dict[int, float]:
        best_score = -math.inf
        best_actions = {vehicle_id: self.min_acceleration for vehicle_id in vehicle_ids}
        for candidate in product(self.candidates, repeat=len(vehicle_ids)):
            action = dict(zip(vehicle_ids, candidate))
            if not self._feasible(action, by_id, edges):
                continue
            score = self.efficiency_weight * sum(candidate)
            for edge in edges:
                if edge.edge_type == "crossing":
                    predicted = self._predicted_delta(edge, action, by_id)
                    if math.isfinite(predicted):
                        score += self.safety_weight * min(predicted, 5.0)
            if score > best_score:
                best_score = score
                best_actions = action
        return best_actions

    def _speed(self, vehicle: VehicleState, acceleration: float) -> float:
        return min(self.speed_limit, max(0.05, vehicle.speed + acceleration * self.horizon / 2))

    def _predicted_delta(
        self,
        edge: ConflictEdge,
        action: dict[int, float],
        by_id: dict[int, VehicleState],
    ) -> float:
        if edge.distance_to_conflict_source is None or edge.distance_to_conflict_target is None:
            return math.inf
        source = by_id[edge.source]
        target = by_id[edge.target]
        source_speed = self._speed(source, action.get(edge.source, source.acceleration))
        target_speed = self._speed(target, action.get(edge.target, target.acceleration))
        return abs(
            edge.distance_to_conflict_source / source_speed
            - edge.distance_to_conflict_target / target_speed
        )

    def _feasible(
        self,
        action: dict[int, float],
        by_id: dict[int, VehicleState],
        edges: list[ConflictEdge],
    ) -> bool:
        for edge in edges:
            if edge.edge_type == "crossing" and edge.critical:
                if self._predicted_delta(edge, action, by_id) < self.min_ttcp_gap:
                    return False
            elif edge.edge_type == "following":
                follower = by_id[edge.source]
                leader = by_id[edge.target]
                follower_position = follower.progress + self._speed(
                    follower, action.get(follower.vehicle_id, follower.acceleration)
                ) * self.horizon
                leader_position = leader.progress + self._speed(
                    leader, action.get(leader.vehicle_id, leader.acceleration)
                ) * self.horizon
                if leader_position - follower_position < self.vehicle_length + self.stop_gap:
                    return False
        return True
