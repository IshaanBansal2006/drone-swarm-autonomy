# 014 — Covariance strategy: square-root UKF

**Status:** Accepted
**Date:** 2026-07-29

## Context

The standard UKF's measurement update (`P⁺ = P − K S Kᵀ`) is not guaranteed to keep `P`
positive semi-definite under floating-point error and strong measurement nonlinearity. This
bit twice in practice: a live crash on first real data (`LinAlgError: Matrix is not positive
definite`) and a test-suite counterexample where the *stored* `P` went indefinite
(eigenvalue −0.03) from a wide prior. Interim guards (symmetrization + eigenvalue-floor
repair) were in place; the question (backlog D-B2) was the permanent strategy. A separately
discovered sigma-point bug (see `docs/ukf-sigma-point-bug.md`) turned out to be a major
contributor to the original instability.

## Options considered

- **Shortcut + guards (status quo):** `P − K S Kᵀ`, symmetrize, floor negative eigenvalues on
  violation. Simplest; repair is reactive, not structural.
- **Joseph-analog form:** `P⁺ = P − K Pxzᵀ − Pxz Kᵀ + K S Kᵀ` — algebraically identical when
  `K = Pxz S⁻¹` is exact, but quadratic in `K` → first-order insensitive to gain error.
  Zero extra cost. Still not PSD-*guaranteed*.
- **Square-root UKF** (Van der Merwe & Wan 2001): carry the Cholesky factor `S` (`P = SSᵀ`)
  through predict/update via QR + rank-1 update/downdate. `P` cannot be indefinite by
  construction (residual failure: downdates, counted + repaired). ~30% more per-step cost.

## Bake-off data (benchmarks/covariance_strategies.py, 20 seeds × 300 steps)

| scenario | strategy | repairs | rmse_pos | rmse_vel | NEES | µs/step |
|---|---|---|---|---|---|---|
| nominal | all three | 0 | 0.056 | 0.273 | ~9,784 | 1690/1640/2115 |
| stress | shortcut | 0 | 0.591 | 0.539 | 115,961 | 1710 |
| stress | joseph | 0 | 0.460 | 0.440 | 115,594 | 1743 |
| stress | **srukf** | 0 | **0.440** | 0.450 | **95,010** | 2243 |

Post-sigma-fix, repairs were zero everywhere — the strategies differ only at the margins:
tie nominal; SR-UKF best under stress; Joseph a free second.

## Decision

**Square-root UKF as the default** (`UKFConfig.covariance_form = "srukf"`), constructed via
`edge.filters.make_filter()`. Shortcut and Joseph remain selectable on the standard UKF for
comparison and teaching.

Rationale: structural PSD safety and the best stress-case accuracy/consistency are
worth ~30% per-step cost at this problem size (n=9, ~20 µs-scale updates — nowhere near the
budget). The bake-off harness stays in-repo so the choice can be re-litigated with data.

## Consequences

- **State carrier changes:** SR-UKF tracks `SRTrackState(x, S)`; `P` is `S @ Sᵀ` on demand.
  `edge.filters.initial_state()` hides the difference from call sites. Downstream consumers
  that need `P` (gating Mahalanobis, NEES, JPDA likelihoods) compute it from the factor —
  the update's returned innovation covariance `S` (measurement-space) is unchanged.
- **`ukf.py` is retained** — as the bake-off baseline, a teaching artifact (the war stories
  live in its explanations), and the Joseph/shortcut reference implementation.
- **Interview surface:** be ready to derive why `P − KSKᵀ` can go indefinite, what Joseph
  form buys (quadratic in K), and how SR-UKF's QR/cholupdate pipeline replaces the
  outer-product covariance reconstruction.
- **Known open issue D-B11 (unaffected by this choice):** NEES ≫ n — the filter is
  overconfident, dominated by extent (`Q_ext` tuning). Strategy choice does not fix
  consistency; tuning does.
