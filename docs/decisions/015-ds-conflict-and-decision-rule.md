# 015 — Dempster-Shafer conflict handling and decision rule

**Status:** Accepted
**Date:** 2026-07-29
**Resolves:** backlog D-B3 (Zadeh mitigation) and D-B4 (decision rule) — the two calls
decision `012` (Dempster-Shafer classification fusion) explicitly deferred.

## Context

Dempster's rule combines evidence by intersecting focal sets and renormalizing away the
mass that lands on the empty set ("conflict" K, divided out as 1/(1−K)). **Zadeh's paradox:**
when two confident sources disagree (camera `{truck}:0.99`, IR `{person}:0.99`, each 0.01 on
`{car}`), K ≈ 0.9999 and renormalization awards near-certainty to `car` — a class *neither*
source supported. Separately, fused belief functions give interval-valued support
`[Bel, Pl]` per class; emitting a single class label requires a decision rule.

## D-B3 — conflict handling: options considered

- **Yager's rule** — route conflict mass to Θ (total ignorance) instead of renormalizing.
  *Pros:* principled ("disagreement means we don't know"), conservative, simple.
  *Cons:* non-associative (fusion order changes results); long fusion chains can end nearly
  vacuous.
- **Conflict-weighted discounting** *(chosen)* — before combining, discount each source
  toward ignorance in proportion to how much it conflicts with the other sources (Shafer
  discounting `m^α`), then apply standard Dempster.
  *Pros:* models **per-sensor reliability** explicitly — the exact physical situation
  (a jammed radar, a glare-blinded camera *should* lose voting power); once discounted,
  Dempster's rule keeps its associativity; degrades gracefully (α→1 recovers pure Dempster).
  *Cons:* largest design surface — needs a discount-factor policy (see below).
- **Murphy's average-then-combine** — average the mass functions, then self-combine n−1
  times. *Pros:* simple, symmetric, dilutes outliers. *Cons:* ad hoc; averaging before
  combining abandons the independent-evidence semantics that motivate DS in the first place.

**Decision: conflict-weighted discounting.** Rationale: it best matches
the project's heterogeneous-sensor reality — the system should attribute disagreement to an
unreliable *source* rather than declare global ignorance (Yager) or blur the evidence
(Murphy) — and the reliability-weighting story is the defensible one for a defense-autonomy
narrative (sensor health → trust). The extra design surface is accepted as worth exploring.

**Discount-factor policy (implementation sub-choice, revisitable):** each source `i` is
discounted by `α_i = 1 − K̄_i`, where `K̄_i` is the mean pairwise Dempster conflict between
`m_i` and each other source. Shafer discounting:
`m^α(A) = α·m(A)` for A ≠ Θ, `m^α(Θ) = 1 − α·(1 − m(Θ))`.
Alternatives left open: static per-sensor reliability priors from config; conflict with the
leave-one-out combination instead of mean pairwise; time-smoothed α (sensor health memory).

## D-B4 — decision rule: options considered

- **Max-plausibility** — argmax Pl. Optimistic: "least ruled out" wins; can pick a class
  with almost no committed support. (Was the stub's placeholder docstring.)
- **Max-belief** — argmax Bel. Conservative: only proven support counts; freezes under
  ignorance (everything ties at 0).
- **Pignistic transform (BetP)** *(chosen)* — redistribute each focal set's mass uniformly
  over its members, producing a probability distribution; argmax it.
  *Pros:* the standard principled bridge from belief functions to a *decision* (Smets'
  transferable-belief model: credal level for reasoning, pignistic level for acting);
  yields a well-behaved confidence number for free — directly consumable by the L3 COP
  display and the L4 approve-gate. *Cons:* flattens the Bel/Pl interval at decision time —
  the "how uncertain" signal must be read from the interval, not from BetP alone.

**Decision: pignistic.** Rationale: principled middle ground between the optimistic
and conservative extremes, and the emitted `(class, confidence)` pair is exactly what the
downstream layers need.

## Consequences

- `classification.py` implements: per-sensor mass construction (camera/lidar → singleton
  focal elements; radar → broad categories; no label → vacuous Θ), pairwise conflict,
  Shafer discounting, Dempster combination (with a logged near-total-conflict guard),
  Bel/Pl, and `decide()` via BetP.
- Zadeh's case becomes regression-tested behavior: vanilla Dempster produces the paradox;
  the discounted pipeline must not.
- **Interview surface:** derive Dempster's rule; walk Zadeh's paradox; explain *why*
  discounting (reliability semantics) over Yager (ignorance semantics); pignistic vs
  max-Bel/max-Pl; BetP's uniform-split assumption.
- Wiring into the tracker is blocked on one remaining micro-decision: **D-B7** (canonical
  `class_beliefs` type on `Track`).
