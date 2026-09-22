# Game in Graph

Game in Graph is a compact research implementation for inspecting the
state-conflict traffic graph (SCTG), its community updates, and cooperative
decisions in small traffic simulations.

This repository accompanies the Game in Graph manuscript. It is an initial
research preview intended for method inspection and lightweight testing.

## Project website

The source for the project website is in [`website/`](website/). GitHub Pages
deployment is defined in [`.github/workflows/deploy-pages.yml`](.github/workflows/deploy-pages.yml).
Once GitHub Pages is enabled for this repository, the page will be available at
`https://fangshiyuu.github.io/Game-in-Graph/`.

> This repository provides a compact research preview of Game in Graph for method inspection and lightweight testing. The full evaluation configurations, extended experiment scripts, and complete reproducibility package will be added after publication.

## Current scope

The preview includes:

- vehicle-state updates and route-based motion;
- SCTG construction with following and crossing interactions;
- conflict weights, community detection, and event-driven community refreshes;
- a compact cooperative decision loop;
- fixed-seed intersection, roundabout, and merging demonstrations;
- trajectory CSV, diagnostics, summary metrics, and a small result figure.

It does not include the full-scale evaluation configurations, all random-seed
results, comparison methods, training pipelines, figure-production scripts, or
hardware/communication integrations. This preview should therefore not be used
to claim reproduction of every numerical result, figure, or statistical
conclusion in the manuscript.

For the release boundary, see [docs/release_scope.md](docs/release_scope.md).

## Supported scenarios

- `intersection`
- `roundabout`
- `merging`

## Layout

```text
configs/                 Small fixed-seed demonstration settings
src/game_in_graph/       Method and simulation modules
scripts/run_demo.py      Command-line simulation entry point
examples/                Minimal Python invocation
tests/                   Automated smoke and graph checks
docs/                    Data format and implementation map
results/                 Local generated outputs (ignored by Git)
```

The implementation map is available in [docs/code_mapping.md](docs/code_mapping.md),
and the output schema is documented in [docs/data_format.md](docs/data_format.md).

## Requirements and installation

Python 3.10 or newer is recommended.

```bash
python -m venv .venv
# Windows
.venv\Scripts\activate
# macOS/Linux
# source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-dev.txt
```

The default `enumeration` backend has no commercial-solver requirement.

### Optional Gurobi backend

This release also provides a compact Gurobi linear-programming backend for
small solver-backed subgroup decisions. Install Gurobi and obtain a valid
license from [Gurobi](https://www.gurobi.com/), then install its Python package:

```bash
python -m pip install -r requirements-gurobi.txt
```

No Gurobi license, license path, or credential is distributed with this
repository. The optional backend is for the lightweight preview configurations;
it is not a distribution of the full experimental optimization setup.

## Quick start

Run the default lightweight demonstration:

```bash
python scripts/run_demo.py --scenario intersection --config configs/intersection_demo.yaml
```

Run the same compact case with the optional Gurobi backend:

```bash
python scripts/run_demo.py --scenario intersection --config configs/intersection_demo.yaml --solver gurobi
```

Choose a custom output location with `--output`:

```bash
python scripts/run_demo.py --scenario roundabout --config configs/roundabout_demo.yaml --output results/roundabout_trial
```

Each run writes:

- `trajectories.csv`: vehicle state at every simulation step;
- `edges.csv` and `communities.csv`: SCTG and community-update diagnostics;
- `metrics.json`: status, safety, and efficiency metrics;
- `trajectory.png`: a simple trajectory/progress visualization.

The command-line summary reports simulation status, vehicle count, SCTG edge
count, communities, a minimum safety-related measure, completion count, and
average delay.

## Configuration

Scenario YAML files contain the seed, time settings, vehicles, graph thresholds,
community parameters, and solver settings. The `solver.backend` field selects
`enumeration` (default) or `gurobi`; `--solver` overrides it without modifying
the YAML file. Keep the provided cases small when using the enumeration backend,
since it searches a discrete action grid.

## Testing

```bash
python -m compileall src scripts
python -m pytest -q
```

The smoke tests load all three configurations, complete short simulations,
verify generated outputs, and check that key metrics are finite.

## Citation

The manuscript is not yet formally published. Please use the software-release
metadata in [CITATION.cff](CITATION.cff) and update the citation after the
publication record is available.

## Contact

Please open a GitHub issue for repository questions.

## License status

No open-source license has been selected for this pre-publication release.
Copyright is retained by the authors; reuse or redistribution requires prior
permission.
