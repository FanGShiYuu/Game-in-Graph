# Data format

## Configuration vehicle record

Each item in `vehicles` has:

| Field | Type | Meaning |
| --- | --- | --- |
| `id` | integer | Persistent vehicle identifier used by graphs and communities |
| `kind` | string | `cav` or `hdv` |
| `route` | string | Scenario route identifier |
| `progress` | float | Arc length from the route origin in metres |
| `speed` | float | Longitudinal speed in metres per second |
| `spawn_step` | integer | Control step at which the vehicle enters |

The internal `VehicleState` additionally stores acceleration and accumulated delay. Cartesian position and heading are derived from route progress.

The legacy source used an index-based list:

```text
[x, y, speed, heading, distance_to_goal, entrance, exit,
 vehicle_id, previous_acceleration, vehicle_kind, plan_deviation]
```

The preview replaces list indices with named fields but keeps persistent IDs, route semantics, SI units, and the verified kinematic ordering.

## Output files

`trajectories.csv` contains one row per active vehicle per control step. Distances are metres, time is seconds, speed is metres per second, acceleration is metres per second squared, and heading is radians.

`edges.csv` contains one row per SCTG edge per step. `delta_ttcp_s` is empty for car-following edges. `critical` marks edges promoted to the forced conflict weight.

`communities.csv` records every community at every step together with the current partition revision and trigger reason.

`metrics.json` uses JSON `null` when no finite crossing-conflict TTCP difference was observed; it never emits `NaN` or machine-local paths.
