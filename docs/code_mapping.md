# Implementation map

This preview keeps the public method path small and explicit.

| Public module | Responsibility |
| --- | --- |
| `config.py` | Load and validate YAML demonstration settings. |
| `vehicle.py` | Vehicle state, route progress, and bounded kinematic updates. |
| `scenarios/` | Fixed lightweight intersection, roundabout, and merging routes. |
| `interaction_graph.py` | Build SCTG nodes and following/crossing interaction edges. |
| `community.py` | Detect communities and refresh them after graph events. |
| `solver.py` | Enumerated or optional Gurobi cooperative subgroup decisions. |
| `simulation.py` | Simulation loop, graph/community updates, and result collection. |
| `metrics.py` | Safety and efficiency summary measures. |

`scripts/run_demo.py` is the single command-line entry point for the three
demonstration scenarios. Scenario size, seed, graph thresholds, and solver
settings are parameterized in `configs/*.yaml`; there are no separate
parameter-only main programs.

The modules expose the sequence used in the preview:

```text
scenario state -> SCTG -> community update -> subgroup decision -> motion update -> metrics
```
