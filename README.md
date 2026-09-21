# Game in Graph

A compact, inspectable implementation of the Game in Graph (GIG) cooperative-driving workflow for mixed traffic.

This repository accompanies the unpublished manuscript *Game in Graph: Distributed Cooperative Driving Framework for Multi-Level Equilibrium in Mixed Traffic*. It exposes the verified research-code path for vehicle propagation, spatiotemporal conflict graph construction, conflict-weighted community detection, dynamic repartitioning, and lightweight subgroup decision inspection.

> This repository provides a compact research preview of Game in Graph for method inspection and lightweight testing. The full evaluation configurations, extended experiment scripts, and complete reproducibility package will be added after publication.

This preview is not a full reproduction package. It must not be used to claim reproduction of every number, figure, statistical conclusion, large-scale experiment, or field test in the manuscript.

## Included scope

- Schematic intersection, roundabout, and merging-area demos using the route identifiers and scenario categories present in the research code.
- Vehicle state, path-following kinematics, CAV/HDV labels, and fixed-parameter IDM behavior for HDVs.
- SCTG nodes and edges based on route conflict points, time to conflict point (TTCP), and the verified source weight `exp(3 - delta_ttcp)`.
- A forced high-conflict weight for critical TTCP differences and car-following relations.
- Leiden modularity partitioning and repartition triggers caused by vehicle-set changes or critical conflicts.
- A small, deterministic, safety-constrained action-enumeration backend for inspecting the subgroup decision flow.
- Safety/efficiency summary metrics, trajectory CSV output, graph snapshots, community history, and one trajectory plot.
- Fixed-seed smoke tests that require no source edits.

The lightweight decision backend is intentionally bounded to a small action grid. It preserves the verified acceleration limits, conflict order, and receding-horizon data flow, but it is not the full experimental optimizer. The original experimental code uses a larger mixed-integer Gurobi formulation, while the current manuscript describes an SQP formulation. Neither is silently substituted here. See [docs/code_mapping.md](docs/code_mapping.md) and [docs/audit.md](docs/audit.md).

## Not included

- The approximately 5,000-interaction evaluation batches, all traffic-flow/penetration combinations, or all random seeds.
- MAPPO, MADQN, iDFST, Auction, CGIG, or other baseline implementations.
- Training resources, field-test communication/hardware interfaces, Redis/OBU integration, or real-vehicle data.
- Paper plotting, table-generation, rebuttal, review, and experiment-search scripts.
- The full Gurobi optimization model, solver debug artifacts, or any personal Gurobi license.
- A verified implementation of the manuscript's EMA edge smoothing, constrained mixed-edge Leiden formulation, SQP equations, MOBIL lane-changing policy, or online HDV preference-vector update. These items are described in the manuscript but were not all present in the audited active simulator path.

## Supported scenarios

The demo geometry is deliberately small and schematic; it is not the evaluation map used to generate manuscript results.

| Scenario | Config | Demonstrated interaction |
| --- | --- | --- |
| Intersection | `configs/intersection_demo.yaml` | Crossing and car-following conflicts |
| Roundabout | `configs/roundabout_demo.yaml` | Entry/circulating-route conflicts |
| Merging area | `configs/merging_demo.yaml` | Mainline/ramp merging and following |

## Repository layout

```text
configs/                 Fixed-seed demo configurations
docs/                    Audit, mapping, and data-format notes
examples/                Minimal Python API example
scripts/                 Command-line demo and plotting entry points
src/game_in_graph/       Core implementation
tests/                   Smoke and graph-behavior tests
results/                 Generated outputs (ignored by Git)
```

## Requirements and installation

Python 3.10 or newer is recommended.

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# Linux/macOS: source .venv/bin/activate
python -m pip install -r requirements.txt
# Contributors running tests can additionally install:
python -m pip install -r requirements-dev.txt
```

`igraph` and `leidenalg` implement the Leiden partition used by the preview. Gurobi is not required for the lightweight demos. To run the unreleased full experimental optimizer, a separate Gurobi installation and valid license are required; no license file or license path is distributed here.

## Quick start

From the repository root:

```bash
python scripts/run_demo.py --scenario intersection --config configs/intersection_demo.yaml
python scripts/run_demo.py --scenario roundabout --config configs/roundabout_demo.yaml
python scripts/run_demo.py --scenario merging --config configs/merging_demo.yaml
```

Each command prints simulation status, vehicle count, SCTG edge count, detected communities, the minimum TTCP-difference safety measure, completion count, and average delay. Output is written beneath `results/<scenario>/`.

Run the API example and tests with:

```bash
python examples/minimal_example.py
python -m compileall src scripts
python -m pytest -q
```

## Configuration

Each YAML file contains:

- `simulation`: fixed seed, number of steps, control period, planning horizon, and output directory.
- `vehicle`: dimensions, acceleration limits, speed limit, and standstill gap.
- `graph`: free-agent distance, TTCP thresholds, critical weight, and Leiden seed.
- `solver`: the small acceleration candidate set and inspection-objective weights.
- `vehicles`: route, type (`cav` or `hdv`), initial progress/speed, and spawn step.

Vehicle state and output schemas are documented in [docs/data_format.md](docs/data_format.md).

## Outputs

The demo produces:

- `trajectories.csv`: time-indexed vehicle states and assigned community.
- `edges.csv`: SCTG edges, edge type, weight, TTCP difference, and critical flag.
- `communities.csv`: community membership whenever a partition is evaluated.
- `metrics.json`: status, counts, minimum safety measure, collision count, completion, throughput, and average delay.
- `trajectory.png`: a compact path/trajectory visualization.

Generated files under `results/` are ignored by Git.

## Known limitations and manuscript/code differences

The audited simulator stores vehicle state in an index-based list, uses semi-implicit longitudinal propagation, computes unsmoothed TTCP edge weights, runs Leiden on an undirected weighted graph, and overrides HDV plans with fixed IDM behavior. The manuscript has since described EMA-smoothed weights, directed co-assignment constraints, SQP, MOBIL, and adaptive HDV preference estimation. Those later descriptions cannot be verified as one integrated path in the supplied source. This preview therefore exposes the verified behavior and flags the differences instead of fabricating equivalence.

The inspection solver is suitable for short, low-vehicle-count examples only. It does not establish the paper's full numerical performance or theoretical claims. Detailed findings are in [docs/audit.md](docs/audit.md).

## Citation

Citation metadata is provided in [CITATION.cff](CITATION.cff). The manuscript is currently listed as unpublished; no DOI, journal volume, issue, or acceptance status is asserted.

## Contact

For research questions, contact Shiyu Fang at `fangshiyu@tongji.edu.cn` or open a GitHub issue.

## License status

No open-source license has been granted for this preview. Copyright is reserved by the authors. Public visibility does not by itself grant permission to copy, modify, or redistribute the software. A formal license will be selected separately after the authors complete the publication and third-party-rights review.
