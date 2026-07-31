# 016 — Joint data association (JPDA)

**Status:** STOPPED 2026-07-31 at the project pivot — enumeration implemented and tested, never
integrated. See the stop note at the end.
**Date:** 2026-07-30
**Amends:** `011` (data association), which chose JPDA and deferred the *joint* layer

## Context

`011` chose JPDA and `tracker.py` shipped the **PDA** computation: each track normalises its
association hypotheses in isolation. That was exact, because the sim had **one target** — with a
single track there is nothing to compete over, and joint JPDA is literally the same function.

The multi-target scene removes that excuse. The concrete failure independent PDA produces:

> Two targets pass close. Both gates cover both detections. Track A takes 70% of detection 1 and
> track B *also* takes 70% of detection 1 — physically impossible, since one detection came from
> one object. Both filters get pulled toward the same target, one track starves and is deleted,
> and when the targets separate the survivor follows the wrong one.

The estimate stays accurate and becomes accurate *about the wrong object*. For a system whose
output feeds an engagement proposal, that is the worst available failure mode, and it is invisible
to any metric that only measures position error.

## Decision

**Enumerate feasible joint events, marginalise each track's betas out of the joint distribution.**
An event assigns each track either "not detected" or one gated detection, subject to the exclusion
constraint that no detection is claimed twice.

```
P(event) ∝ ∏ P_D · N_t(z_assigned)  ·  ∏ (1 − P_D·P_G)  ·  λ^(detections left as clutter)
             assigned                    unassigned
β_t(j) = Σ P(event)   over events where track t took detection j
```

### Solver: Murty's k-best (author's choice), with exhaustive enumeration as the oracle

| Option | Exactness | Cost | Verdict |
|---|---|---|---|
| Exhaustive enumeration | Exact | Factorial in tracks × overlapping detections | **Shipped now**; kept permanently as the test oracle |
| Murty's k-best | Approximate (top-k events, remainder truncated) | Polynomial; k × Hungarian | **Chosen** for the shipped path |
| Cheap/suboptimal JPDA | Approximate, no enumeration | Cheapest | Rejected — gives up exactness *and* the ability to check itself |

**Author's rationale:** scale. Murty is what production trackers use and what keeps the door open
to scenes far larger than today's.

**The counter-argument, recorded because it is real:** at 2–5 targets, exhaustive enumeration is
*exact* and Murty is *approximate* — truncating at k discards low-probability events that
enumeration keeps. So in the near term Murty is strictly the worse estimator, chosen for a
scalability property the current scenes do not exercise. This is accepted deliberately.

**What makes it safe:** enumeration is not thrown away. It stays as the exactness oracle — Murty's
top-k events must match enumeration's top-k on small problems, and the marginal betas must agree
to within the truncated mass. Without that oracle, a subtly wrong k-best implementation produces
plausible numbers forever. This is the same pattern as the SR-UKF being test-pinned against the
standard UKF (`014`).

### Failure behaviour

`JPDAConfig.max_events` (default 10,000) bounds enumeration. On overflow the code falls back to
independent PDA, **increments `event_overflows`, and logs a warning naming the fix**. It never
truncates silently — a silently-truncated event set produces betas that look normal and are wrong.

## Consequences

- `TrackGate(gated_idx, likelihoods)` carries **global** detection indices so conflicts between
  tracks are detectable; `gaussian_likelihood` is shared so the PDA and JPDA paths cannot drift.
- Output layout is identical to the PDA path (`betas[0]` = null, `betas[k+1]` = k-th gated), so
  `compute_combined_innovation` and everything downstream is unchanged. The joint upgrade is a
  drop-in at the call site.
- **`tracker.py` must invert its loop.** It currently iterates track-major (for each track, gate
  and update against every sensor). Joint association needs sensor-major: gather *all* tracks'
  predictions for one sensor, associate jointly, then apply per-track updates. That restructure is
  the integration step.
- Still simplified, and still logged in the backlog: the pseudo-measurement update omits PDA's
  covariance-inflation ("spread of innovations") term, so the filter stays optimistic in clutter.
  Joint association fixes *which* detection a track believes in, not how uncertain it should be
  about having chosen.

## Open

- **Assignment solver source** for Murty: `scipy.optimize.linear_sum_assignment` (adds a heavy
  dependency to a numpy+pydantic project) vs a hand-written Hungarian (~100 lines, no dependency,
  and squarely the kind of algorithm this project exists to be able to defend). Undecided.

---

## Stop note — 2026-07-31: work halted at the project pivot

**Status changed to: implemented and tested, never integrated. Deliberately stopped.**

The project pivoted to a focused research question — decentralized VLM-driven semantic coverage
under communication and perception faults — whose pipeline is
`pixels → VLM → importance field → consensus → coverage control → motion`. There is no
multi-object tracking anywhere in it, so the whole L1 association stack is out of scope.

**What exists and is green:**

- `edge/jpda.py` — exhaustive feasible-event enumeration with marginalisation, overflow guard,
  and the fallback-with-warning path. 7 tests.
- `assignment.py` — hand-written Hungarian (shortest augmenting path with potentials), pinned
  against brute-force permutation across 7 matrix shapes. 12 tests.

**What was queued and is NOT built:** Murty's k-best (the solver above was written for it), the
seeded multi-target scene, the HOTA harness, and the `tracker.py` sensor-major restructure that
would have made joint association run live.

**Why the enumerator is not wired in.** Integration required inverting the tracker's loop from
track-major to sensor-major (see Consequences above). That restructure was the next step when the
pivot landed. So `016`'s algorithm is correct, tested, and *dormant* — a decision doc describing a
capability the running system does not use. Recorded here rather than quietly deleted, because
the alternative is a repo whose docs claim more than its code does.

**What carries into the new project:** nothing from this decision directly. The Tier-2 robustness
mechanism there is conflict-weighted Dempster–Shafer discounting (decision `015`), where the
indices move from sensor-vs-sensor to agent-vs-agent. Belief combination is not data association —
it needs neither the Hungarian nor the joint enumerator.

**If this is ever resumed**, the order is: Murty behind the same interface with the enumerator kept
permanently as the exactness oracle, then the sensor-major restructure, then the multi-target
scene, then HOTA. The open solver question in the section above is now answered — the hand-written
Hungarian in `assignment.py` was the choice, and it is done.
