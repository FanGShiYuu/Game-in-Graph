# Source-to-preview mapping

The source was audited from `E:/.../2025Dgame/Code`; the local user-specific path is intentionally not stored in executable code or configuration.

| Research-code responsibility | Audited source | Preview module | Disposition |
| --- | --- | --- | --- |
| Global constants and scenario switch | `params.py` | `config.py`, `configs/*.yaml` | Rewritten as portable validated configuration |
| Vehicle state and kinematics | `Vehicle.py`, `Environment_*.py` | `vehicle.py`, `scenarios/*` | Named state model; verified semi-implicit update retained |
| Route geometry and coordinate conversion | `Environment_intersection.py`, `Environment_roundabout.py`, `Environment_merging_area.py` | `scenarios/*` | Rewritten as compact schematic demo routes; evaluation maps deferred |
| Conflict relation and TTCP | `Louvain.py::get_ttcp`, `Environment_*.py` conflict tables | `interaction_graph.py` | Core unsmoothed TTCP and exponential weight retained |
| Community partition | `Louvain.py::communitity_detection` | `community.py` | Leiden weighted modularity retained; deterministic seed added |
| Dynamic update trigger | `main*.py::update_communities` | `CommunityTracker.update` | Node-change and critical-TTCP triggers retained |
| Cooperative optimization | `Vehicle.py::CAV_1D` | `solver.py` | Full Gurobi MIP deferred; explicit bounded inspection backend supplied |
| HDV realization | `main*.py::hdv_update`, `Vehicle.py::IDM` | `vehicle.py`, `simulation.py` | Fixed IDM parameters retained in compact form |
| Simulation entry | duplicate `main20/40/60/80*.py` programs | `scripts/run_demo.py` | Merged into one parameterized entry |
| Metrics and Excel export | `main*.py`, `calculate/*` | `metrics.py`, `simulation.py` | Minimal safety, completion, throughput, delay, and CSV/JSON only |
| Paper plotting and experiment post-processing | `plot/*`, `calculate/*` | Not included | Deferred; only one demo trajectory plot is retained |
| ADMM, priority, baseline, socket, and field-test helpers | `ADMM_solver.py`, `Priority_scheduler.py`, `Node_*.py`, third-party tree | Not included | Outside initial core preview |

The preview does not copy the bundled `HighwayEnv-master`, virtual environment, cache files, model debug files, raw results, or scripts with machine-specific output paths.
