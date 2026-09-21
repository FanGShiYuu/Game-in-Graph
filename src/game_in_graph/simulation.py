"""End-to-end short simulation and portable result export."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import random

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .community import CommunityTracker
from .config import DemoConfig
from .interaction_graph import GraphBuilder, InteractionGraph
from .metrics import collision_pairs
from .scenarios import load_scenario
from .solver import build_solver
from .vehicle import VehicleState, idm_acceleration


@dataclass(frozen=True)
class SimulationResult:
    status: str
    metrics: dict
    output_dir: Path
    final_graph: InteractionGraph
    final_communities: list[list[int]]
    first_actions: dict[int, float]


class Simulation:
    def __init__(self, config: DemoConfig, output_dir: str | Path | None = None) -> None:
        self.config = config
        self.scenario = load_scenario(config.scenario)
        self.dt = float(config.simulation["dt"])
        self.steps = int(config.simulation["steps"])
        self.output_dir = Path(output_dir) if output_dir is not None else config.output_dir
        seed = int(config.simulation["seed"])
        random.seed(seed)
        np.random.seed(seed)
        self.graph_builder = GraphBuilder(
            self.scenario,
            config.graph,
            float(config.simulation["planning_horizon"]),
        )
        self.community_tracker = CommunityTracker(int(config.graph["leiden_seed"]))
        self.solver = build_solver(
            config.solver,
            config.vehicle,
            float(config.simulation["planning_horizon"]),
        )
        self.pending = sorted(config.vehicles, key=lambda item: (int(item["spawn_step"]), int(item["id"])))
        self.vehicles: list[VehicleState] = []
        self.completed: list[VehicleState] = []
        self.seen_ids: set[int] = set()
        self.collision_set: set[tuple[int, int]] = set()
        self.minimum_delta_ttcp = math.inf
        self.trajectory_rows: list[dict] = []
        self.edge_rows: list[dict] = []
        self.community_rows: list[dict] = []
        self.first_actions: dict[int, float] = {}

    def run(self) -> SimulationResult:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        final_graph = InteractionGraph((), (), ())
        final_communities: list[list[int]] = []
        for step in range(self.steps):
            self._spawn(step)
            self._remove_completed()
            final_graph = self.graph_builder.build(self.vehicles)
            final_communities, repartitioned, reason = self.community_tracker.update(final_graph)
            actions = self.solver.solve(self.vehicles, final_graph, final_communities)
            if not self.first_actions:
                self.first_actions = dict(actions)
            actions = self._apply_hdv_behavior(actions, final_graph)
            self._record(step, final_graph, final_communities, repartitioned, reason)
            self._update_metrics(final_graph)
            self._advance(actions)
        self._remove_completed()
        status = "completed" if not self.collision_set else "completed_with_collision"
        metrics = self._summary(status, final_graph, final_communities)
        self._write_outputs(metrics)
        self._plot()
        return SimulationResult(
            status=status,
            metrics=metrics,
            output_dir=self.output_dir.resolve(),
            final_graph=final_graph,
            final_communities=final_communities,
            first_actions=self.first_actions,
        )

    def _spawn(self, step: int) -> None:
        ready = [item for item in self.pending if int(item["spawn_step"]) == step]
        self.pending = [item for item in self.pending if int(item["spawn_step"]) != step]
        for item in ready:
            route_id = str(item["route"])
            self.scenario.route(route_id)
            vehicle = VehicleState(
                vehicle_id=int(item["id"]),
                kind=str(item["kind"]).lower(),
                route_id=route_id,
                progress=float(item["progress"]),
                speed=float(item["speed"]),
                spawn_step=int(item["spawn_step"]),
            )
            if vehicle.kind not in {"cav", "hdv"}:
                raise ValueError(f"Vehicle {vehicle.vehicle_id} has invalid kind {vehicle.kind!r}.")
            self.vehicles.append(vehicle)
            self.seen_ids.add(vehicle.vehicle_id)

    def _remove_completed(self) -> None:
        remaining = []
        for vehicle in self.vehicles:
            if vehicle.progress >= self.scenario.route(vehicle.route_id).length - 5.0:
                self.completed.append(vehicle)
            else:
                remaining.append(vehicle)
        self.vehicles = remaining

    def _apply_hdv_behavior(self, actions: dict[int, float], graph: InteractionGraph) -> dict[int, float]:
        adjusted = dict(actions)
        by_route: dict[str, list[VehicleState]] = {}
        for vehicle in self.vehicles:
            by_route.setdefault(vehicle.route_id, []).append(vehicle)
        for route_vehicles in by_route.values():
            route_vehicles.sort(key=lambda item: item.progress, reverse=True)
        for vehicle in self.vehicles:
            if vehicle.kind != "hdv":
                continue
            lane = by_route[vehicle.route_id]
            index = lane.index(vehicle)
            front = lane[index - 1] if index > 0 else None
            gap = None if front is None else front.progress - vehicle.progress - float(self.config.vehicle["length"])
            action = idm_acceleration(vehicle, front, gap)
            for edge in graph.edges:
                if edge.edge_type != "crossing" or vehicle.vehicle_id not in {edge.source, edge.target}:
                    continue
                other_id = edge.target if edge.source == vehicle.vehicle_id else edge.source
                other = next(item for item in self.vehicles if item.vehicle_id == other_id)
                vehicle_distance = (
                    edge.distance_to_conflict_source
                    if edge.source == vehicle.vehicle_id
                    else edge.distance_to_conflict_target
                )
                other_distance = (
                    edge.distance_to_conflict_target
                    if edge.source == vehicle.vehicle_id
                    else edge.distance_to_conflict_source
                )
                if vehicle_distance is not None and other_distance is not None:
                    vehicle_ttcp = vehicle_distance / max(vehicle.speed, 0.05)
                    other_ttcp = other_distance / max(other.speed, 0.05)
                    if abs(vehicle_ttcp - other_ttcp) < float(self.config.graph["critical_ttcp"]):
                        if vehicle_ttcp >= other_ttcp and self.config.scenario != "roundabout":
                            action = float(self.config.vehicle["min_acceleration"])
            adjusted[vehicle.vehicle_id] = action
        return adjusted

    def _advance(self, actions: dict[int, float]) -> None:
        self.vehicles = [
            vehicle.advance(
                actions[vehicle.vehicle_id],
                self.dt,
                float(self.config.vehicle["speed_limit"]),
                float(self.config.vehicle["min_acceleration"]),
                float(self.config.vehicle["max_acceleration"]),
            )
            for vehicle in self.vehicles
        ]

    def _record(
        self,
        step: int,
        graph: InteractionGraph,
        communities: list[list[int]],
        repartitioned: bool,
        reason: str,
    ) -> None:
        membership = {
            vehicle_id: index for index, community in enumerate(communities) for vehicle_id in community
        }
        for vehicle in self.vehicles:
            route = self.scenario.route(vehicle.route_id)
            x, y, heading = route.pose_at(vehicle.progress)
            self.trajectory_rows.append(
                {
                    "step": step,
                    "time_s": step * self.dt,
                    "vehicle_id": vehicle.vehicle_id,
                    "kind": vehicle.kind,
                    "route_id": vehicle.route_id,
                    "x_m": x,
                    "y_m": y,
                    "heading_rad": heading,
                    "progress_m": vehicle.progress,
                    "distance_to_goal_m": route.length - vehicle.progress,
                    "speed_mps": vehicle.speed,
                    "acceleration_mps2": vehicle.acceleration,
                    "community": membership.get(vehicle.vehicle_id, -1),
                }
            )
        for edge in graph.edges:
            self.edge_rows.append(
                {
                    "step": step,
                    "source": edge.source,
                    "target": edge.target,
                    "edge_type": edge.edge_type,
                    "weight": edge.weight,
                    "delta_ttcp_s": edge.delta_ttcp,
                    "critical": edge.critical,
                }
            )
        for index, community in enumerate(communities):
            self.community_rows.append(
                {
                    "step": step,
                    "revision": self.community_tracker.revision,
                    "repartitioned": repartitioned,
                    "reason": reason,
                    "community": index,
                    "vehicle_ids": ";".join(str(item) for item in community),
                }
            )

    def _update_metrics(self, graph: InteractionGraph) -> None:
        finite = [
            edge.delta_ttcp
            for edge in graph.edges
            if edge.delta_ttcp is not None and math.isfinite(edge.delta_ttcp)
        ]
        if finite:
            self.minimum_delta_ttcp = min(self.minimum_delta_ttcp, min(finite))
        collisions = collision_pairs(
            self.vehicles,
            self.scenario,
            float(self.config.vehicle["length"]),
            float(self.config.vehicle["width"]),
        )
        self.collision_set.update(collisions)

    def _summary(self, status: str, graph: InteractionGraph, communities: list[list[int]]) -> dict:
        all_vehicles = self.vehicles + self.completed
        average_delay = sum(vehicle.delay for vehicle in all_vehicles) / max(len(all_vehicles), 1)
        duration = self.steps * self.dt
        return {
            "status": status,
            "scenario": self.config.scenario,
            "solver_backend": self.solver.backend,
            "seed": int(self.config.simulation["seed"]),
            "steps": self.steps,
            "duration_s": duration,
            "number_of_vehicles": len(self.seen_ids),
            "active_vehicles": len(self.vehicles),
            "number_of_sctg_edges": len(graph.edges),
            "detected_communities": communities,
            "community_revisions": self.community_tracker.revision,
            "minimum_delta_ttcp_s": (
                None if not math.isfinite(self.minimum_delta_ttcp) else self.minimum_delta_ttcp
            ),
            "collision_count": len(self.collision_set),
            "completed_vehicles": len(self.completed),
            "throughput_vehicles_per_hour": len(self.completed) / duration * 3600.0,
            "average_delay_s": average_delay,
            "first_control_actions_mps2": self.first_actions,
        }

    def _write_outputs(self, metrics: dict) -> None:
        self._write_csv("trajectories.csv", self.trajectory_rows)
        self._write_csv("edges.csv", self.edge_rows)
        self._write_csv("communities.csv", self.community_rows)
        with (self.output_dir / "metrics.json").open("w", encoding="utf-8") as handle:
            json.dump(metrics, handle, indent=2, sort_keys=True, allow_nan=False)

    def _write_csv(self, name: str, rows: list[dict]) -> None:
        path = self.output_dir / name
        if not rows:
            path.write_text("", encoding="utf-8")
            return
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    def _plot(self) -> None:
        figure, axis = plt.subplots(figsize=(6, 6))
        for route in self.scenario.routes.values():
            axis.plot(route.points[:, 0], route.points[:, 1], color="0.8", linewidth=4)
        colors = plt.colormaps["tab10"]
        vehicle_ids = sorted(self.seen_ids)
        for index, vehicle_id in enumerate(vehicle_ids):
            rows = [row for row in self.trajectory_rows if row["vehicle_id"] == vehicle_id]
            if rows:
                axis.plot(
                    [row["x_m"] for row in rows],
                    [row["y_m"] for row in rows],
                    label=f"Vehicle {vehicle_id}",
                    color=colors(index % 10),
                )
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_title(f"Game in Graph: {self.config.scenario} demo")
        axis.legend(loc="best", fontsize=8)
        figure.tight_layout()
        figure.savefig(self.output_dir / "trajectory.png", dpi=150)
        plt.close(figure)
