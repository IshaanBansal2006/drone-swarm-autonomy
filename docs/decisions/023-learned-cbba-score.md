# 023 — Learned path score inside CBBA

**Status:** Accepted — open sub-decisions listed under Consequences
**Date:** 2026-09-13

## Context

`050` adds a learned component as a capability extension. After `018`, each drone carries a pose
covariance and the swarm shares a prior map of road signs.

CBBA's score (`_path_score` in `autonomy/allocator.py`) is a time-discounted reward along a
straight-line path. It is blind to localization: a drone whose pose has drifted bids exactly as a
well-localized one does, and a route past a mapped sign is valued no differently from one that
passes none.

## Options considered

- **Learned follow behaviour** (`_follow_leaf`). Single-drone and easy to see, but every training step
  runs the full filter.
- **Localization-aware follow.** As above, plus it needs radar coverage gaps; a reward built on filter
  covariance is exploitable wherever the filter is inconsistent (D-B11 recorded extent NEES of 9,770
  against an ideal of 3).
- **Learned camera pointing.** Needs a gimbal the scene does not have.
- **Learned score inside CBBA.** ← chosen. Swarm-level; bundle construction and consensus stay
  hand-written; the prior sign map makes predicted localization quality computable at allocation time.
  Harder to show visually, and localization-aware scores are not diminishing-marginal-gain in general.
- **Dropped:** learned low-level control (the kinematic backend leaves nothing to learn); a learned
  coverage planner (that seam plans once rather than deciding sequentially); a learned allocator
  replacing CBBA (unjustifiable without a comparison, which `050` rules out).

## Decision

**Replace CBBA's hand-written path score with a learned score that accounts for predicted
localization quality along each drone's path, keeping bundle construction and consensus unchanged.**

## Consequences

- `_path_score` becomes a pluggable score function passed to `CBBAAllocator`; the time-discounted
  score stays as the default.
- **Open — convergence.** `allocator.py` documents diminishing marginal gain as the property CBBA's
  convergence guarantee needs. Passing a sign early can make later tasks more valuable, which breaks
  it. Either enforce it by construction (localization may only discount a score, never raise it —
  guarantee kept, that synergy lost) or accept its loss and rely on the round cap, stating plainly
  that convergence is no longer proven.
- **Open — learning formulation.** (i) a value estimate fit to rollout returns under CBBA — policy
  evaluation, honestly labelled a learned value function; (ii) policy gradient on score parameters
  through the non-differentiable allocation — RL control, high variance; (iii) approximate policy
  iteration alternating (i) with reallocation.
- Training runs without Isaac: kinematic backend, synthesized measurements, and an allocation-time
  localization-quality predictor built from the prior sign map rather than the full filter per step.
- Capability claim only: the README describes localization-aware allocation, not an improvement over
  the hand-written score.
- Depends on `018`. Ships under tag `video-5-*`.
