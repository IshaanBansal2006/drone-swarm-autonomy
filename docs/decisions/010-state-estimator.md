# 010 — State estimator

**Status:** Accepted
**Date:** 2026-07-13

## Context

L1 needs a filter to maintain track state (position, velocity, covariance) and ingest measurements from three heterogeneous sensor types (decision 007): EO/IR camera (pixel-space bounding boxes), radar (polar range-bearing-doppler), and lidar (cartesian point cloud). Each sensor has a nonlinear observation model `h(x)`. The filter choice determines how those nonlinearities are handled and what assumptions are made about the state distribution.

## Options considered

- **EKF:** First-order linearization via hand-derived Jacobians. Industry workhorse. Cheap. Degrades with radar's polar nonlinearity at close range.
- **UKF:** Second-order accuracy via sigma points. No Jacobians needed — just write `h(x)` per sensor. Slightly more expensive. Better nonlinear handling.
- **IMM:** Multiple motion models in parallel (CV/CA/CT), weighted by likelihood. Layer on top of EKF or UKF. Correct for maneuvering targets. N× cost.
- **Particle filter:** Arbitrary distributions. Expensive. Degeneracy in 6-9D state. Overkill for this nonlinearity level.

## Decision

**UKF (Unscented Kalman Filter).**

Primary motivator: three heterogeneous observation models. UKF avoids deriving and maintaining three separate Jacobian matrices (one per sensor type). Adding a new sensor = write `h(x)`, done. The radar polar→cartesian nonlinearity is better handled by sigma points than by first-order linearization.

IMM is left as a future upgrade if sim targets exhibit complex maneuvering (constant velocity → coordinated turn → stop) and the single-model UKF can't keep up. The upgrade path is clean: IMM wraps multiple UKF sub-filters.

## Consequences

- **User implements:** UKF predict/update cycle, sigma point generation (Van der Merwe or Julier), process noise tuning, three observation functions `h(x)` for camera/radar/lidar.
- **No Jacobians to derive.** Each new sensor type is a single function `h(x)` mapping state → observation space.
- **Tuning surface:** sigma point parameters (alpha, beta, kappa) + process noise Q + per-sensor measurement noise R. Start with textbook defaults, tune via NIS/NEES consistency checks.
- **IMM upgrade path preserved.** If tracking performance degrades on maneuvering targets, wrap UKF instances in an IMM framework without rewriting the observation models.
- **Gaussian assumption retained.** Cannot represent multi-modal target distributions. Acceptable for this problem class.
