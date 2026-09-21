# Initial public-preview audit

Audit date: 2026-09-21

## Verified active research-code behavior

- The duplicated `main20.py`, `main40.py`, `main60.py`, `main80.py`, and `*-pri.py` variants are byte-identical in several groups. Their node-count names are not reliable configuration evidence; the active behavior comes from shared global parameters and runtime state.
- The simulation control step is `DT = 0.1 s`; the planning buffer uses a separate `DT_PLANNING` after the first step.
- Vehicle dimensions are `4 m × 2 m`; speed and acceleration limits in the supplied parameter file are `10 m/s`, `-4 m/s²`, and `2 m/s²`.
- Community membership is stored by persistent vehicle ID, not mutable list position.
- `Louvain.py::get_ttcp` computes crossing TTCP as distance-to-conflict divided by speed, applies `exp(TTCP_THR - delta_ttcp)`, and promotes `delta_ttcp < 0.5 s` to a large forced weight.
- The active partition implementation converts the adjacency to an undirected graph and calls `leidenalg.find_partition(..., ModularityVertexPartition)`.
- Candidate TTCP relations are checked during active conflict cycles, but the stored adjacency/partition is rebuilt only after vehicle-set changes or a critical TTCP condition.
- HDV actions are realized through fixed IDM parameters. The planned cooperative action is retained only as a deviation signal in the legacy state.
- The CAV optimizer in `Vehicle.py` imports Gurobi, creates acceleration and binary variables, and writes `model.ilp`/`model.mps` when infeasible. No license file is required or retained by this preview.

## Manuscript/source discrepancies

These differences are recorded rather than silently resolved:

1. The manuscript states that SCTG weights use an EMA with `alpha = 0.3` and are updated every `0.1 s`. The audited graph implementation uses instantaneous TTCP differences and does not retain an EMA state.
2. The manuscript defines a mixed graph where directed following edges become hard co-assignment constraints and are excluded from modularity. The audited active partition builds one undirected weighted adjacency matrix.
3. The manuscript describes nonlinear subgroup optimization with SQP and smooth comfort reward. The supplied `Vehicle.py` active convex path uses a Gurobi mixed-integer model with binary collision-order variables and a different implemented reward expression.
4. The manuscript describes an explicit three-element preference vector and an online HDV preference update. The audited simulator has no integrated preference-vector state or update call; HDV realization uses fixed IDM.
5. The manuscript names IDM and MOBIL. IDM is present in the active simulator; a verified active MOBIL lane-change policy was not found.
6. The manuscript kinematic equation includes `0.5 * a * dt²` in position propagation. The active path-following implementation updates speed first and then advances distance using the updated speed.
7. The supplied parameter snapshot selects the `priority` engine, while the paper method is reached only when the engine setting is changed to `game`. File names alone do not identify an experimental run.

## Public disposition

### Retained or rewritten

- Vehicle state and kinematic update.
- Fixed-parameter IDM HDV behavior.
- Three scenario categories and route-based conflict points.
- TTCP edge construction, verified edge weight, critical-weight promotion, Leiden partition, and dynamic update triggers.
- A transparent low-dimensional decision backend, metrics, CSV/JSON output, plot, and smoke tests.

### Merged

- Repeated node-count, traffic-flow, penetration-rate, and engine-specific main programs are represented by one CLI plus YAML configuration.
- Scenario-independent simulation flow is centralized; scenario geometry remains isolated by module.

### Excluded

- Virtual environments, caches, IDE state, trash/history, Excel temporary files, raw experiment results, solver debug files, large figures/videos, bundled HighwayEnv, comparison/training algorithms, socket helpers, plot/table pipelines, rebuttal/review material, and local-path experiment searches.
- Third-party source with unclear redistribution status. Dependencies are installed from their own distributions.

### Deferred until publication

- Full evaluation maps/configurations, complete experimental solver, all baselines, large-scale seeds and results, field-test interfaces, and the complete reproducibility package.
- Integration and validation of EMA SCTG updates, directed co-assignment constraints, SQP, MOBIL, and online preference estimation as one coherent code path.

## Validation boundary

The public tests validate deterministic imports, all three short demos, file generation, finite values, collision checks, and the verified TTCP weight formula. A full frame-by-frame numerical comparison against the original experimental stack is not currently possible without selecting a historical engine/parameter snapshot and a valid Gurobi runtime. Such equivalence is therefore marked **not verifiable**, not assumed.
