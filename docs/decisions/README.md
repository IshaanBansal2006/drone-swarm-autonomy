# Decision Log

Every non-trivial decision — scope, sim environment, algorithm choice, message schema — lives here as an ADR.

## Format

Each doc:

```markdown
# NNN — Title

**Status:** Proposed | Accepted | Superseded by NNN
**Date:** YYYY-MM-DD

## Context
Why this decision is on the table.

## Options considered
List with pros/cons.

## Decision
The one chosen. In one sentence.

## Consequences
What now flows from this — what's easier, what's harder, what's ruled out.
```

## Index

| # | Title | Status |
|---|---|---|
| 000 | Scope and layer depth | Accepted |
| 001 | Sim environment | Accepted (Isaac Sim on Windows + ROS2 bridge to WSL) |
| 002 | Roadmap slot / timing | Pending |
| 003 | Success target / venue | Pending |
| 004 | Category structure | Pending |
| 005 | Intent-parser tech | Pending (constrained by 001 — see doc) |
| 006 | Real hardware stretch | Pending (cheap given 001) |
| 007 | Sensor modalities | Accepted (radar + EO/IR + lidar, phased, mixed topology, heterogeneous loadouts) |
| 010 | State estimator | Accepted (UKF; IMM upgrade path preserved) |
| 011 | Data association | Accepted (JPDA first; MHT fallback if needed) |
| 012 | Classification fusion | Accepted (Dempster-Shafer belief functions) |
| 020 | Task decomposer | Ready to decide after reading |
| 021 | Allocator | Ready to decide after reading |
| 022 | Coverage planner | Ready to decide after reading |
| 030 | 3D viewer | Ready to decide |
| 031 | Data transport | Partially resolved by 001 (ROS2 for sim ↔ L1/L2); COP frontend transport still open |
| 040 | Interface schemas | Scoped to ROS2 .msg / IDL types by 001; still needs field-level design |
