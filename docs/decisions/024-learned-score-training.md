# 024 — Training the learned CBBA score: unconstrained score, approximate policy iteration, counterfactual credit

**Status:** Accepted
**Date:** 2026-09-13
**Depends on:** `023` (learned path score), `019`/`041` (pose estimate with covariance on `DroneState`)

## Context

`023` decided *that* CBBA's path score becomes learned and localization-aware, and left two
things open: what to do about CBBA's convergence guarantee, and what "learned" means. Answering
them raised two more — how a shared mission return is attributed to one drone's bids, and what
the return is. All four are recorded here.

## Options considered

**Convergence (CBBA's proof needs diminishing marginal gain, DMG)**
- **Enforce DMG by construction** — localization may only discount a bid. Guarantee kept; the
  effect worth learning (a sign fix early makes later tasks worth *more*) is exactly what DMG
  forbids.
- **Accept losing the guarantee** ← chosen. Bids may rise with expected localization gain; the
  round cap and bundle truncation end the auction. Convergence becomes an empirical claim and
  must be measured.

**Learning formulation**
- **Value estimate fit to rollouts** — supervised, stable, honestly a learned value function;
  no policy improvement.
- **Policy gradient through the allocation** — real RL; high variance (one return per mission,
  discrete non-differentiable allocation, shared credit).
- **Approximate policy iteration** ← chosen. Alternate fitting a value/advantage to rollouts of
  the current score with replacing the score. Improvement without the gradient's variance; no
  general convergence guarantee, so it needs a stopping rule and a held-out check.

**Credit assignment (a return is per mission; bids are per drone)**
- **Per-drone decomposition** — a drone's own task rewards. Simple; ignores that its bids move
  the other drones' bundles.
- **Counterfactual** ← chosen. `credit_d = R(all on the new score) − R(d on the reference, others
  on the new score)`, both rollouts under the **same seed** (common random numbers), so the only
  difference is d's bids. The marginal contribution of d's policy, coupling included; one extra
  rollout per drone per mission.

**Counterfactual reference**
- **Frozen classical score** — stable meaning; the signal saturates once the learned score is
  clearly better.
- **Previous iterate** ← chosen. Signal stays informative; the reference drifts, so the frozen
  classical score is still evaluated alongside for reporting.

**Mission return**
- CBBA's own objective (time-discounted rewards collected) — ignores localization.
- Rewards minus a weighted localization loss — a hand-tuned weight to defend.
- **Rewards weighted by localization quality at completion** ← chosen. A task done while badly
  localised counts for less. No weight; `quality(var) = 1 / (1 + var / s0²)` with `s0 = 1 m`
  (continuous, half credit at 1 m² of position variance — the one derived constant here).

## Decision

**Bids may violate diminishing marginal gain; the score is trained by approximate policy
iteration on counterfactual credit against the previous iterate under common random numbers;
the return is the sum of task rewards weighted by localization quality at completion.**

Structure that follows: `score = baseline + w·φ`, where `baseline` is the hand-written
time-discounted score (DMG intact) and `w·φ` is a linear advantage on features from an
allocation-time localization predictor (drift with distance, reset near a mapped sign). Because
the credit is measured against the previous iterate it is an *improvement* signal: the fitted
weights are **added** to the current ones, so at a fixed point the credit vanishes and the
weights stop moving. Each iteration reports held-out return against the previous iterate and the
frozen classical score, and the fraction of allocations that converged before the round cap;
training stops on no held-out improvement, a convergence-rate collapse, or the budget.

## Found while implementing

The shipped CBBA never had the guarantee strictly. Diminishing marginal gain holds for the
time-discounted score when tasks are *appended*, which is the setting of the proof; the
allocator inserts each task at its *best position*, and there a task already in the path can act
as a paid-for detour into a neighbour's area and raise that neighbour's marginal gain.
`tests/test_learned_score.py` pins both: DMG exact under append, violated in a minority of
random 4-task cases under best insertion. So "accept losing the guarantee" costs less than the
options table implies — the round cap was already doing the work — and the convergence-rate
measurement now describes the classical score too, not only the learned one.

**The previous-iterate reference is degenerate at the start.** Credit is `R(new) − R(previous)`
under common random numbers; with `new == previous` (iteration 1 starts from the classical
score) the two rollouts are identical and the credit is exactly zero — the same property the
common-random-numbers test relies on. The first training run fitted zero for four iterations.
Iteration 1 therefore starts from a small random perturbation of the classical score
(`TrainConfig.explore_std`, 0.1 in feature units); from iteration 2 the loop is as stated. This
is the standard observation that policy iteration needs a non-degenerate starting policy, not a
change to the credit rule.

## Consequences

- `CBBAAllocator` takes a pluggable `PathScore` (and per-drone overrides for the
  counterfactual); it records `last_converged` / `last_rounds`.
- New: `autonomy/learned_score.py` (predictor, features, `LearnedScore`, JSON persistence),
  `autonomy/training.py` (mission rollouts, credit, iteration), `benchmarks/learned_score.py`
  (trains and writes `config/learned_score.json` + a report). The mission node loads the weights
  if present.
- Rollouts run the real ego filter on a synthesised IMU with sign fixes — signs only, no vehicle
  landmarks, for speed. Targets and the tracker are not in the loop: the return does not depend on
  them.
- **Capability claim only** (`050`): the README reports the measured held-out return and the
  convergence rate; it does not claim the learned score beats the classical one beyond what those
  two numbers show.
- The predictor is a heuristic (linear drift, sign-range reset, no field-of-view) and the
  advantage is linear in seven features. Both are the simplest thing that can carry the signal;
  either can be replaced behind the `PathScore` seam.
- Ships under tag `video-5-learned-score`.
