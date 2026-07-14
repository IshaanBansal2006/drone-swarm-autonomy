# 012 — Classification fusion

**Status:** Accepted
**Date:** 2026-07-13

## Context

Different sensors produce different classification signals. Camera outputs a visual class label + confidence (e.g., "truck 0.87"). Radar produces speed and RCS profiles that imply broad categories (e.g., "large slow-mover"). Lidar gives shape geometry that distinguishes structural classes. L1 must fuse these into a single class estimate per track, accounting for the fact that each sensor has different classification strengths, different failure modes, and may sometimes produce no classification at all.

## Options considered

- **Naive Bayes:** multiply per-sensor class likelihoods, normalize. Simple. Independence assumption is wrong (sensors observe the same target). Over-confident.
- **Bayesian with confusion matrices:** model each sensor's classification bias via `P(observed | true)`. Principled, interpretable. Needs calibration data to estimate confusion matrices. Doesn't natively handle "no opinion."
- **Dempster-Shafer (belief functions):** explicitly represents uncertainty AND ignorance. Mass functions assigned to subsets of the class set. Combination rule fuses evidence from multiple sources. Handles "radar has no opinion" naturally (assign mass to the full frame of discernment). Known pathology: Zadeh's paradox with highly conflicting evidence.
- **Weighted voting:** ad hoc, no principled uncertainty handling. Too simple to defend in an interview.

## Decision

**Dempster-Shafer theory (belief functions).**

Primary motivator: the three sensor types have fundamentally different classification capabilities. Radar may have no opinion on fine-grained class. Lidar may distinguish "vehicle" from "person" but not vehicle subtypes. Camera has the richest class vocabulary but fails in degraded visibility. Dempster-Shafer's explicit representation of ignorance ("I have no evidence" ≠ "uniform probability") maps directly to this heterogeneous capability landscape.

## Consequences

- **User implements:** mass function construction per sensor type, Dempster's combination rule, belief/plausibility computation, decision rule (e.g., max plausibility, or belief threshold).
- **Zadeh's paradox is a known risk.** When two sensors produce highly conflicting evidence (camera says "truck" with high confidence, lidar says "person" with high confidence), Dempster's rule produces counterintuitive results (all mass goes to a third class). Mitigations to study and choose from:
  - Yager's modified rule (assigns conflict mass to ignorance instead of normalizing it away)
  - Conflict-weighted discounting (reduce a sensor's mass function when it conflicts with the consensus)
  - Murphy's average-then-combine (average mass functions before combining to dilute outlier evidence)
  - The mitigation choice is itself an interview talking point.
- **Mass function design per sensor:**
  - Camera: focal element = specific class, mass proportional to classifier confidence, remainder to full frame.
  - Radar: focal elements = broad categories (e.g., {ground vehicle}, {air vehicle}, {person}), mass from RCS + doppler profile, high mass to full frame (radar is often ignorant on fine class).
  - Lidar: focal elements = shape-derived categories, mass from geometric fit confidence.
- **Interview preparation:** be ready to derive Dempster's rule, explain belief vs. plausibility, walk through Zadeh's paradox and your chosen mitigation, and compare to Bayesian fusion. The "why not Bayesian?" answer: Bayesian cannot distinguish "I don't know" from "uniform prior" — Dempster-Shafer can, and that distinction matters when radar literally cannot classify beyond broad categories.
- **Independence between 012 and 010/011.** Classification fusion runs on class labels attached to measurements after association (011) assigns them to tracks. It does not affect the state estimator (010) or the association logic.
