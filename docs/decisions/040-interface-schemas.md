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
  position_cov (3x3 row-major), class_label, class_confidence, class_beliefs (DS mass,
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
