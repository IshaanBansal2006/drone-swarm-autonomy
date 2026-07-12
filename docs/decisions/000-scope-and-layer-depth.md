# 000 — Scope and layer depth

**Status:** Accepted
**Date:** 2026-07-11

## Context

Mini-Lattice is inspired by Anduril's Lattice OS: one operator, N autonomous vehicles + heterogeneous sensors, unified operating picture, human-on-the-loop safety. A full Lattice is not buildable by one person in a reasonable timeframe. A "mini-Lattice" needs a scoping decision: which layers go deep enough to defend on a whiteboard, and which stay shallow enough to demo but not defend.

## Options considered

**Option A — Edge Layer only** (deep MOT + fusion, stubs for autonomy/COP/HOL)
- Pros: Deepest single-topic depth; publishable; matches classical robotics/fusion labs.
- Cons: Doesn't demonstrate the Lattice paradigm; not enough surface area for autonomy roles.

**Option B — Mission Autonomy only** (deep multi-agent planning, stubs elsewhere)
- Pros: Fits DeepMind Robotics / CCA-style multi-agent labs; publishable.
- Cons: Weak sensor-fusion story; less unique.

**Option C — Full stack, thin** (all four layers shallow)
- Pros: Demonstrates the paradigm end-to-end.
- Cons: Nothing whiteboard-defensible; reads as "systems generalist," weak for research-lab targets.

**Option D — L1–3 deep, L4 light** (chosen)
- L1 Distributed Edge: deep — MOT, sensor fusion math, classification fusion.
- L2 Mission Autonomy: deep — intent parsing, task decomposition, multi-agent allocation, replanning.
- L3 COP: moderate — 3D operator terminal, real-time updates, sensor overlays. Demo-critical, not whiteboard fodder.
- L4 Human-on-the-Loop: light — approve-to-engage state machine + audit log + safety narrative.
- Pros: Two interview-defensible layers (perception + planning); demo-visible COP; safety story without over-investing.
- Cons: 4+ months minimum; scope risk high; not roadmap-native (roadmap targets research labs; this project skews defense/product).

## Decision

**Option D.** Layers L1 and L2 go deep enough to defend algorithmically on a whiteboard. L3 goes deep enough to look good in a demo video and be architecturally defensible. L4 is a state machine + narrative, no advanced algorithms.

## Consequences

- Two interview cores to prepare (perception + planning), not one.
- Project directly targets defense-autonomy (Anduril, Shield AI) and hybrid product-autonomy roles (Skydio, Waymo planning, Wayve). Weaker signal for pure research labs (PI, DeepMind Robotics) than P3/P4 would be.
- COP must exist to sell the demo; can't be skipped.
- L4 safety narrative is Ishaan's to write — not scaffoldable by AI.
- All algorithm choices within L1/L2/L3 remain open (decisions `010`–`031`).
- Cross-layer message schemas must be locked before layer implementation begins (decision `040`).
- Adjacent decisions still pending: sim env (`001`), timing (`002`), venue (`003`), category structure (`004`), intent-parser tech (`005`), real-hardware stretch (`006`).
