# 022 — Coverage planner: Voronoi partitioning (frontier-ready)

**Status:** Accepted
**Date:** 2026-07-30

## Context

The `scan` intent (L2-d fork) needs area coverage: split a region among drones and sweep
it. Blocked the HTN's `_method_scan` until decided.

## Options considered

- **Boustrophedon / lawnmower** — serpentine sweep of the whole area, split into strips.
  *Pros:* simplest, provably complete on known free space. *Cons:* centralized strip
  assignment; "uninteresting" (design doc's word) — no multi-agent structure.
- **Voronoi partitioning** *(chosen)* — partition the area into cells by drone seed
  positions (each point belongs to its nearest drone); each drone sweeps its own cell.
  *Pros:* elegant spatial decomposition; naturally decentralized/scalable (each drone's
  region derives from geometry, mirroring the CBBA/consensus story); balances travel by
  construction; strong interview material (Lloyd's algorithm, coverage control literature —
  Cortés et al.). *Cons:* more geometry; cells change as drones move (we partition once per
  scan intent, not continuously).
- **Frontier-based exploration** — greedily target the boundary between known and unknown
  space. *Pros:* THE approach for **unknown maps**. *Cons:* needs an occupancy map being
  built online — overkill while the sim map is fully known.
- **Learned coverage policy** — belongs, if ever, in the Step-4/5 learned stack.

## Decision

**Voronoi partitioning**, implemented **grid-based**: the area polygon is discretized into
sample points; each point is assigned to its nearest drone seed (a discrete Voronoi
partition); each cell's points are ordered into a serpentine route (intra-cell routing is an
implementation detail, not the 022 decision) and emitted as one coverage task per cell.
CBBA (021) then allocates the region-tasks — proximity scoring naturally hands each drone
its own cell.

**The unknown-map switch (user requirement, planned):** I intends to move to unknown
maps with physical hardware later. The grid representation is chosen PRECISELY because
frontier exploration is also grid-based (occupancy grid): the planner sits behind a
`CoveragePlanner` protocol, and swapping `VoronoiCoverage` → `FrontierCoverage` reuses the
same grid machinery + task interface. Known map = partition once (Voronoi); unknown map =
re-plan toward frontiers as the map grows. The seam is the same one 020 uses for the LLM.

## Consequences

- `autonomy/coverage.py`: `CoveragePlanner` protocol + `VoronoiCoverage`; HTN's
  `_method_scan` delegates to the injected planner.
- Grid resolution is a knob (coarse default; tune per scene scale).
- Static partition per intent: cells are seeded from drone positions at decompose time;
  continuous Lloyd-style re-centering is future work if scan quality demands it.
- Interview surface: Voronoi coverage control vs boustrophedon completeness vs frontier
  exploration; why grid discretization unifies them.
