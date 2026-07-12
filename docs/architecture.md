# Architecture

The authoritative design lives in the [Notion project page](https://app.notion.com/p/399ca937182781478f0ccef0ac8be7a4). This file mirrors the current architecture summary and is updated as decisions land.

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

## Cross-layer message schemas

**To be defined** (see `docs/decisions/040-interface-schemas.md`). Interface stability > implementation elegance — freeze schemas before writing layer code.

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
