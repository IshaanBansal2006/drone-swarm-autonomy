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
| 001 | Sim environment | Accepted (Isaac Sim on Windows + ROS 2 bridge to WSL; Foxy references obsolete after the 20.04→22.04 / Humble migration — see the addendum) |
| 007 | Sensor modalities | Accepted (radar + EO/IR + lidar, phased, mixed topology, heterogeneous loadouts) |
| 008 | Project vision and staged build plan | Superseded by 009 from step 3 onward |
| 009 | Conclude the platform, pursue the coverage question | Accepted (project closed 2026-07-31) |
| 010 | State estimator | Accepted (square-root UKF; IMM upgrade path preserved) |
| 011 | Data association | Accepted (JPDA chosen; PDA shipped — joint layer amended by 016) |
| 012 | Classification fusion | Accepted (Dempster-Shafer belief functions) |
| 013 | Observation and sensing | Accepted |
| 014 | Covariance strategy | Accepted (benchmarked, see `benchmarks/`) |
| 015 | DS conflict and decision rule | Accepted (conflict-weighted discounting + BetP) |
| 016 | Joint data association (JPDA) | **Stopped at the pivot** — enumeration and Hungarian built and tested, never integrated |
| 020 | Task decomposer | Accepted (HTN) |
| 021 | Allocator | Accepted (CBBA) |
| 022 | Coverage planner | Accepted (Voronoi) |
| 030 | COP viewer and transport | Accepted (Rerun) |
| 040 | Interface schemas | Accepted (pydantic cross-layer schemas) |

Numbers 002–006 and 031 were reserved during planning and never written. 002 (roadmap slot),
003 (success venue) and 004 (category structure) were answered by 008 and then by 009; 005
(intent-parser tech) and 006 (real hardware) were withdrawn with 008's steps 3–5; 031 (data
transport) was resolved inside 001 and 030.
