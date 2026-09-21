"""Cooperative decision backends for compact Game in Graph demonstrations."""

from __future__ import annotations

from itertools import product
import math
from typing import Protocol

from .interaction_graph import ConflictEdge, InteractionGraph
from .vehicle import VehicleState


class SolverUnavailableError(RuntimeError):
    """Raised when an explicitly selected optional solver cannot be used."""


class PreviewSolver(Protocol):
    """Interface shared by the lightweight cooperative decision backends."""

    backend: str

    def solve(
        self,
        vehicles: list[VehicleState],
        graph: InteractionGraph,
        communities: list[list[int]],
    ) -> dict[int, float]: ...


class BasePreviewSolver:
    """Shared state and graph utilities for preview decision backends."""

    backend = "base"

    def __init__(self, config: dict, vehicle_config: dict, planning_horizon: float) -> None:
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
            actions.update(self._solve_group(controlled, by_id, local_edges))
        return actions

    def _solve_group(
        self,
        vehicle_ids: list[int],
        by_id: dict[int, VehicleState],
        edges: list[ConflictEdge],
    ) -> dict[int, float]:
        raise NotImplementedError

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


class EnumerationSolver(BasePreviewSolver):
    """Enumerate a small action grid under conflict-order safety constraints."""

    backend = "enumeration"

    def __init__(self, config: dict, vehicle_config: dict, planning_horizon: float) -> None:
        super().__init__(config, vehicle_config, planning_horizon)
        self.candidates = tuple(float(item) for item in config["acceleration_candidates"])

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


class GurobiSolver(BasePreviewSolver):
    """Solve compact subgroup LPs with Gurobi for the preview configurations."""

    backend = "gurobi"

    def __init__(self, config: dict, vehicle_config: dict, planning_horizon: float) -> None:
        super().__init__(config, vehicle_config, planning_horizon)
        try:
            import gurobipy as gp
            from gurobipy import GRB
        except ImportError as error:
            raise SolverUnavailableError(
                "The Gurobi backend requires gurobipy. Install requirements-gurobi.txt "
                "and configure a valid Gurobi license."
            ) from error
        self.gp = gp
        self.GRB = GRB

    def _solve_group(
        self,
        vehicle_ids: list[int],
        by_id: dict[int, VehicleState],
        edges: list[ConflictEdge],
    ) -> dict[int, float]:
        model = self.gp.Model("game_in_graph_preview")
        model.Params.OutputFlag = 0
        accelerations = {
            vehicle_id: model.addVar(
                lb=self.min_acceleration,
                ub=self.max_acceleration,
                vtype=self.GRB.CONTINUOUS,
                name=f"a_{vehicle_id}",
            )
            for vehicle_id in vehicle_ids
        }
        model.setObjective(
            self.efficiency_weight * self.gp.quicksum(accelerations.values()),
            self.GRB.MAXIMIZE,
        )
        for edge in edges:
            self._add_safety_constraint(model, accelerations, edge, by_id)
        model.optimize()
        if model.Status != self.GRB.OPTIMAL:
            raise RuntimeError(
                f"Gurobi preview subgroup solve ended with status {model.Status}; "
                "adjust the compact demonstration configuration."
            )
        return {vehicle_id: float(variable.X) for vehicle_id, variable in accelerations.items()}

    def _acceleration_expression(
        self,
        vehicle: VehicleState,
        accelerations: dict[int, object],
    ) -> object:
        return accelerations.get(vehicle.vehicle_id, vehicle.acceleration)

    def _predicted_progress(
        self,
        vehicle: VehicleState,
        accelerations: dict[int, object],
    ) -> object:
        acceleration = self._acceleration_expression(vehicle, accelerations)
        return vehicle.progress + vehicle.speed * self.horizon + 0.5 * acceleration * self.horizon**2

    def _add_safety_constraint(
        self,
        model: object,
        accelerations: dict[int, object],
        edge: ConflictEdge,
        by_id: dict[int, VehicleState],
    ) -> None:
        source = by_id[edge.source]
        target = by_id[edge.target]
        if edge.edge_type == "following":
            follower, leader = source, target
            model.addConstr(
                self._predicted_progress(leader, accelerations)
                - self._predicted_progress(follower, accelerations)
                >= self.vehicle_length + self.stop_gap,
                name=f"following_{edge.source}_{edge.target}",
            )
            return
        if not edge.critical:
            return
        if edge.distance_to_conflict_source is None or edge.distance_to_conflict_target is None:
            return
        source_ttcp = edge.distance_to_conflict_source / max(source.speed, 0.05)
        target_ttcp = edge.distance_to_conflict_target / max(target.speed, 0.05)
        early, late = (source, target) if source_ttcp <= target_ttcp else (target, source)
        early_distance = (
            edge.distance_to_conflict_source if early.vehicle_id == source.vehicle_id else edge.distance_to_conflict_target
        )
        late_distance = (
            edge.distance_to_conflict_source if late.vehicle_id == source.vehicle_id else edge.distance_to_conflict_target
        )
        early_remaining = early_distance - (
            self._predicted_progress(early, accelerations) - early.progress
        )
        late_remaining = late_distance - (
            self._predicted_progress(late, accelerations) - late.progress
        )
        model.addConstr(
            late_remaining - early_remaining
            >= self.min_ttcp_gap * max(min(early.speed, late.speed), 0.05),
            name=f"crossing_{edge.source}_{edge.target}",
        )


def build_solver(config: dict, vehicle_config: dict, planning_horizon: float) -> PreviewSolver:
    """Construct the explicitly selected preview decision backend."""

    backend = str(config.get("backend", "enumeration")).lower()
    if backend == "enumeration":
        return EnumerationSolver(config, vehicle_config, planning_horizon)
    if backend == "gurobi":
        return GurobiSolver(config, vehicle_config, planning_horizon)
    raise ValueError(
        f"Unsupported solver backend {backend!r}. Choose 'enumeration' or 'gurobi'."
    )
