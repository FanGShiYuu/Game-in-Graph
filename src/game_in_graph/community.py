"""Leiden modularity partitioning and source-aligned update triggers."""

from __future__ import annotations

from dataclasses import dataclass

import igraph as ig
import leidenalg as la

from .interaction_graph import InteractionGraph


@dataclass
class CommunityTracker:
    leiden_seed: int = 0
    communities: list[list[int]] | None = None
    previous_nodes: tuple[int, ...] = ()
    revision: int = 0

    def update(self, graph: InteractionGraph) -> tuple[list[list[int]], bool, str]:
        node_change = graph.node_ids != self.previous_nodes
        critical_conflict = any(edge.critical for edge in graph.edges)
        must_partition = self.communities is None or node_change or critical_conflict
        reason = "unchanged"
        if must_partition:
            if self.communities is None:
                reason = "initial"
            elif node_change:
                reason = "vehicle_set_changed"
            else:
                reason = "critical_conflict"
            self.communities = self._partition(graph)
            self.revision += 1
        self.previous_nodes = graph.node_ids
        return [list(group) for group in (self.communities or [])], must_partition, reason

    def _partition(self, graph: InteractionGraph) -> list[list[int]]:
        free = list(graph.free_agent_ids)
        active = [node for node in graph.node_ids if node not in graph.free_agent_ids]
        groups: list[list[int]] = [free]
        if not active:
            return groups
        index = {node: position for position, node in enumerate(active)}
        active_edges = [
            edge
            for edge in graph.edges
            if edge.source in index and edge.target in index and edge.weight > 0
        ]
        if not active_edges:
            # This mirrors the audited no-edge branch, which kept remaining nodes together.
            groups.append(sorted(active))
            return groups
        igraph_graph = ig.Graph(n=len(active), directed=False)
        igraph_graph.vs["vehicle_id"] = active
        igraph_graph.add_edges([(index[edge.source], index[edge.target]) for edge in active_edges])
        igraph_graph.es["weight"] = [edge.weight for edge in active_edges]
        partition = la.find_partition(
            igraph_graph,
            la.ModularityVertexPartition,
            weights="weight",
            seed=self.leiden_seed,
        )
        detected = [sorted(active[item] for item in community) for community in partition]
        detected.sort(key=lambda community: min(community))
        groups.extend(detected)
        return groups
