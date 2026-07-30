# 020 — Task decomposer: HTN planning → BT execution, with an LLM seam

**Status:** Accepted
**Date:** 2026-07-30

## Context

L2 (Step 1 of decision `008`) turns a structured operator intent ("patrol sector B",
"track object 3", "scan area") into a concrete task graph the allocator (`021`) can assign
across drones. This is fork L2-b of the design doc. The choice shapes the planning interview
story, execution robustness, and — critically for this project — the seam where the Step-5
learned layer will eventually plug in.

## Options considered

- **HTN (Hierarchical Task Network)** — abstract tasks recursively decomposed by a
  hand-authored method library until primitive. *Pros:* classic AI planning, explainable,
  whiteboard-defensible ("derive the decomposition"). *Cons:* rigid — every mission recipe
  must be authored; no reactivity at execution time.
- **Behavior Trees** — reactive tick-based composition (sequence/fallback/parallel nodes).
  *Pros:* robotics-industry standard (Nav2, drones, games), composable, handles execution
  *and* monitoring/recovery in one formalism. *Cons:* not a planner — encodes structure, does
  not search; weaker *planning* interview story on its own.
- **LLM decomposition** — a language model emits the task graph. *Pros:* maximally flexible;
  it is where the Step-5 learned layer lives. *Cons:* nondeterministic, no interview math, premature
  before the language front-end (Step 3).
- **PDDL + external planner** — formal action schemas + search. *Pros:* provably correct.
  *Cons:* heavyweight setup for three mission types; poor fit for reactive multi-drone
  execution.

## Decision

**Hybrid: HTN decomposes intents into task graphs; Behavior Trees execute the resulting
tasks per drone. The decomposer sits behind a swappable interface with an explicit LLM slot
for the future.**

Rationale (user, 2026-07-30):
1. **The LLM slot is a certainty, not a maybe** — the Step-5 learned layer (and Step-3 language
   front-end) will need to inject decompositions. Designing the decomposer as a swappable
   component now (the decision-`008` "swappable brain" principle applied one level down)
   means the classical HTN can be *degraded to a baseline* when the learned stack arrives,
   instead of ripped out.
2. **Roadmap order is RL-first, working up** — so Step 1 stays fully classical (HTN), and
   learned components arrive in their planned steps rather than leaking in early.
3. **Best of both formalisms:** HTN carries the *planning* interview story (decomposition
   search, method libraries); BT carries the *execution* story (reactive ticking, fallback
   recovery — the robotics-standard answer). The known cost — building two formalisms — is
   accepted with scope control: a minimal method library (patrol / track / scan) and a
   minimal BT node set.

## Consequences

- `autonomy/` grows: `decomposer.py` (interface + HTN implementation), `bt.py` (minimal
  behavior-tree executor: sequence, fallback, action leaves), `world_state.py` (drone +
  track bookkeeping the methods read).
- **The decomposer interface is the LLM seam:** `Decomposer.decompose(intent, world_state)
  -> TaskGraph`. An `LLMDecomposer` implementing the same interface is scaffolding-ready;
  Step 3/5 fills it. Baseline comparisons (decision `008` ladder) fall out for free.
- Interview surface: HTN semantics (methods, preconditions, recursive decomposition),
  BT semantics (tick, sequence vs fallback, reactivity), and *why hybrid* — the
  plan-then-execute split mirrors deliberative/reactive layered architectures.
- Scope guard: no PDDL, no HTN search beyond first-applicable-method, until a mission type
  demands it.
