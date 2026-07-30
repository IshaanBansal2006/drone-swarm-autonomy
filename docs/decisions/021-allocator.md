# 021 — Multi-agent task allocator: CBBA

**Status:** Accepted
**Date:** 2026-07-30

## Context

L2 must assign the decomposed tasks (`020`) across 2–3 heterogeneous drones (decision
`007`: different sensor loadouts) subject to position/battery/capability constraints. Fork
L2-c of the design doc — flagged there as the L2 "deep piece" (milestone M5) and prime
interview material.

## Options considered

- **Hungarian assignment** — optimal 1-to-1 in O(n³). *Pros:* textbook, exact, fast to
  build. *Cons:* centralized; one task per drone per round; weakest narrative for a
  *distributed* autonomy project.
- **CBBA (Consensus-Based Bundle Algorithm; Choi, Brunet, How 2009)** — decentralized
  market: each drone greedily builds a *bundle* of tasks it bids on; drones exchange bids
  and resolve conflicts by consensus rules until assignment stabilizes. *Pros:* THE
  canonical decentralized swarm-allocation algorithm; matches the project's "Distributed"
  thesis and defense-autonomy positioning (CCA-style swarms); bounded suboptimality
  guarantees; the design doc's designated deep piece. *Cons:* most complex option —
  consensus iterations, conflict-resolution table, careful score functions; harder to debug
  than a solver call.
- **Sequential auction** — greedy market rounds. *Pros:* simple, decent quality. *Cons:* no
  named-algorithm credibility, no optimality story.
- **MILP** — exact optimization. *Pros:* provable optimum. *Cons:* slow, not real-time, and
  a solver dependency; wrong fit for onboard/decentralized narrative.
- **Learned allocator (GNN)** — *Pros:* publishable. *Cons:* training burden; belongs (if
  ever) in the Step-4/5 learned stack, not the classical baseline.

## Decision

**CBBA.**

Rationale: it is the deep, interview-defensible allocator the design doc
planned for, and its decentralized consensus structure is the honest match for the
mini-Lattice thesis (drones that could each run their own allocator, even though the sim
executes centrally for now). Going straight to CBBA rather than a Hungarian-first milestone;
Hungarian remains trivially available later as an evaluation baseline (optimality-gap
measurements for the Step-4/5 ladder) if wanted.

## Consequences

- `autonomy/allocator.py`: CBBA per Choi/Brunet/How — bundle construction (greedy marginal
  score), bid space, consensus/conflict-resolution rules, convergence detection. Score
  function must be **capability-aware** (decision `007`: can't assign a camera-classify task
  to a radar-only drone) and DMG (diminishing marginal gain) to keep CBBA's guarantees.
- Consensus needs a communication model: in Step 1 the "network" is in-process message
  passing between drone agents; the same interfaces carry to real broadcast later.
- Interview surface: CBBA's two phases (bundle build / consensus), why DMG scores are
  required for convergence, suboptimality bound, comparison vs Hungarian/auction/MILP.
- Complexity is the accepted cost; scope guard: 2–3 drones, ≤ ~10 tasks — small enough to
  debug consensus by inspection.
