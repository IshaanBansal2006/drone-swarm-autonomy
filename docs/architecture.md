# Architecture

A summary of the current architecture, updated as decisions land. The reasoning behind each
choice lives in [`docs/decisions/`](decisions/); the full walkthrough is in
[`docs/deep-dive.md`](deep-dive.md).

## Data flow

```
[Sensors (sim)] → L1 (fusion) → track feed → L2 (autonomy) → commands → [Drones (sim)]
                                    ↓              ↓
                                    └─ COP ← ─────┘
                                       (L3)
                                    ↑
                          [Human Operator] → intent → L2
                                    ↑
                              L4 approve-to-engage gate
```

## Runtime topology (per decision 001)

- **Sim:** Isaac Sim on Windows 11 host
- **Code:** L1 / L2 / L3 / L4 in WSL2 Ubuntu 22.04
- **Sim ↔ Code bus:** ROS2 (Humble) across WSL2 mirrored networking (`networkingMode=mirrored`)
- **Sim-to-real path:** swap Isaac Sim publishers for real sensor/state drivers on the same ROS2 topics; L1/L2/L3 unchanged

## Cross-layer message schemas

**To be defined** (see `docs/decisions/040-interface-schemas.md`). Interface stability > implementation elegance — freeze schemas before writing layer code. Scoped to ROS2 `.msg` / IDL types by decision 001.

- `Track` (L1 → L2 & L3): `id`, `class`, `position`, `velocity`, `covariance`, `confidence`, `timestamp`, `source_sensor_ids`
- `Intent` (Operator → L2): natural-language string OR structured goal
- `TaskAssignment` (L2 → sim drones): `drone_id`, `task_type`, `params`, `priority`
- `EngagementProposal` (L2 → L4): `action`, `targets`, `rationale`, `deadline`

## Package layout

```
src/mini_lattice/
  edge/       # L1 — sensor sim, fusion, MOT
  autonomy/   # L2 — intent parser, decomposer, allocator, replanner
  cop/        # L3 — COP backend (frontend is a separate app; TBD)
  hol/        # L4 — approval queue, audit log, safety timeouts
```
