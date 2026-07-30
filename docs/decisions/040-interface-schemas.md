# 040 — Cross-layer interface schemas

**Status:** Accepted (draft approved by user 2026-07-30)
**Date:** 2026-07-30

## Context

The design doc mandates: "Define these BEFORE writing any layer code. Interface stability >
implementation elegance." Decision `001` scoped transport to ROS 2; decision `013`/D-B1 fixed
the L1 output content. Four messages connect the layers. Implementation: **pydantic models**
(source of truth, validation, JSON serialization) carried over ROS 2 — interim as JSON in
`std_msgs/String` (already live on `/tracks`), upgradeable to generated `.msg`/IDL without
changing the pydantic definitions.

## The schemas (as approved)

- **`TrackMsg`** (L1 → L2, L3): `track_id, timestamp, position[3], velocity[3], extent[3],
  position_sqrt_cov (3x3 row-major, see amendment), class_label, class_confidence, class_beliefs (DS mass,
  serialized with frozenset keys as sorted "|"-joined strings), age, source_sensor_ids`.
  *Notable:* `class_beliefs` rides in DS-native form (decision D-B7 carried downstream) so
  L3 can display Bel/Pl intervals, not just a flattened label.
- **`StructuredIntent`** (operator → L2; Step 1 is language-free): `intent_id, verb
  ("patrol"|"track"|"scan"|"goto"), target (track_id | polygon | point), constraints
  (priority, deadline_s, drone_whitelist)`.
- **`TaskAssignment`** (L2 → drones): `task_id, intent_id (provenance chain), drone_id,
  task_type, waypoints[], target_track_id?, params, priority`.
- **`EngagementProposal`** (L2 → L4 gate): `proposal_id, action, target_track_id,
  rationale, deadline_s` (auto-deny on timeout per L4 design).

## Consequences

- Implemented at `src/swarm_autonomy/schemas.py` (top-level — all layers import it; no layer
  owns it).
- The interim `String`+JSON transport is a documented stopgap: pydantic `model_dump_json` /
  `model_validate_json` are the wire codec either way.
- Schema changes from here are **versioned decisions** (amend this doc) — they are the
  expensive-to-change surface (decision `008`: stable interfaces, swappable implementations).

---

## Amendment — 2026-07-30: frame envelope and square-root uncertainty

Promoting `/tracks` from hand-adapted JSON to the real schema surfaced two sub-decisions that
`040` had not settled. Both were decided by the author; options and rationale below.

### A. Frame shape — `TrackFrame` envelope (chosen)

The producer publishes one *complete confirmed picture* per fusion cycle. Three shapes were on
the table:

| Option | Shape | Why not / why |
|---|---|---|
| Envelope | `{timestamp, tracks: [TrackMsg]}` | **Chosen.** |
| Bare list | `[TrackMsg, ...]` | An empty list carries no timestamp. |
| Per-track stream | one message per `TrackMsg` | Loses the frame boundary. |

**Rationale.** The deciding case is the *empty* frame. "L1 is alive and currently confirms
nothing" is a real, actionable operator-facing state, and it is categorically different from
"L1 is dead" — but a bare empty list cannot express it, because it has nowhere to put a
timestamp. Consumers would have to fall back on their last known time, making replay
non-deterministic and hiding a dead producer behind a frozen clock. Both live consumers need
the frame time regardless of track count: `WorldState.update_tracks` advances mission time, and
the Rerun bridge advances the scrubber.

The per-track stream was rejected as premature. It is the natural shape once L1 is genuinely
distributed across drones, but it converts the picture from a **snapshot** to a **delta** — and
deltas require solving track *deletion* explicitly (death messages or a timeout reaper), which
snapshots get for free by simply omitting the track from the next frame. Revisit when multiple
L1 nodes publish concurrently.

### B. Uncertainty payload — Cholesky factor of the position block (chosen)

The field was specified as a covariance and shipped as nine zeros. Options:

| Option | Payload | Verdict |
|---|---|---|
| Marginal 3×3 covariance | `P[:3,:3]`, 9 floats | Rejected — round-trips into non-PSD. |
| Full 9×9 covariance | 81 floats | Rejected by the author as overkill; mostly unread, and its extent block is the least trustworthy part of the state (D-B11). |
| **Cholesky factor** | `L`, `P = L Lᵀ` | **Chosen.** |
| Leave zeros | — | Rejected: a schema field that is structurally a lie is the exact failure the promotion exists to remove. |

**Rationale.** Author's stated intent was to send uncertainty *as needed* rather than shipping
the whole state covariance. The factor form serves that better than it first appears, because of
a property of triangular factors:

> For a lower-triangular `S` with `P = S Sᵀ`, the block `S[:d, d:]` is structurally zero, so
> `P[:d,:d] = S[:d,:d] · S[:d,:d]ᵀ` **exactly**. The leading block of the factor *is* the factor
> of the leading block.

So the 3×3 published today is a free slice of the SR-UKF's existing factor — no re-factorisation
— and widening to 6×6 (position+velocity) or 9×9 later is a pure slice change requiring no
re-derivation and no schema redesign. The property holds only for **leading principal** blocks;
the trailing extent block has no such shortcut.

The second argument is numerical, and this project has already paid for it twice (the α=1e-3
crash and the sigma-point Cholesky bug): a serialized covariance can round-trip into a matrix
that is no longer positive semi-definite, and the consumer discovers this by crashing. A factor
cannot — squaring it reproduces a PSD matrix by construction. Shipping the square root moves an
entire class of failure off the wire.

**Cost, stated plainly.** The 3×3 marginal discards position↔velocity cross-covariance, so a
consumer cannot yet extrapolate a track forward with correct uncertainty or do proper
track-to-track fusion. That is acceptable while L2 consumes tracks only for distance
computations; it is the first thing to widen if L2 ever does its own association.

### Consequences

- `TrackFrame` added to `schemas.py`; `TrackMsg.position_cov` → `position_sqrt_cov`.
- `filters.sqrt_cov_block(state, dim=3)` is the single producer-side accessor. SR-UKF slices;
  standard UKF factors, and on failure floors the spectrum and **logs a warning naming the fix**
  rather than publishing silently-wrong numbers.
- `mission_node._on_tracks` and `cop/rerun_bridge._on_tracks` now validate a `TrackFrame`. The
  short-key adapter (`id`→`track_id`, `class`→`class_label`) is deleted: a producer/consumer
  field mismatch now fails loudly at the boundary instead of silently defaulting.
- The transport is still JSON in `std_msgs/String`; only the **content** was promoted. A custom
  `.msg`/IDL still needs a colcon package and remains future work.
