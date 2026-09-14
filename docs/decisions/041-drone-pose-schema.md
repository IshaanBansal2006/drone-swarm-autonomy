# 041 — Drone pose on the wire: one schema for truth and estimate

**Status:** Accepted
**Date:** 2026-09-13
**Amends:** `040` (interface schemas) — adds the platform-pose messages

## Context

Until now a drone's pose crossed the wire as a bare JSON dict of positions on `/drone_poses`,
produced by L2's kinematics and consumed by the simulator (to render) and the operating picture.
There was no orientation, because nothing needed one, and no estimate, because pose was truth by
construction.

`019` adds two things that need a schema: an orientation (a body-mounted camera has a direction),
and an *estimated* pose with uncertainty produced by L1, distinct from the truth that L2's backend
owns. `DroneState` in L2 carries only `position`.

## Options considered

- **Extend the bare dict** with an orientation and add a second ad-hoc dict for estimates.
  Cheapest; repeats the unvalidated-key problem `040` was written to remove.
- **Two schemas** — a truth message and an estimate message. Explicit, but the two would share
  every field except the covariance, and every consumer would carry two code paths.
- **One schema, covariance optional.** ← chosen. `DronePoseMsg` carries id, time, position and
  orientation; `pose_sqrt_cov` is present on the estimate feed and `None` on the truth feed. A
  `DronePoseFrame` envelope mirrors `TrackFrame` so an empty frame still carries time.

## Decision

**Add `DronePoseMsg` and `DronePoseFrame` to the cross-layer schemas; `/drone_poses` carries truth
from L2 and `/drone_pose_estimates` carries L1's estimate, both as `DronePoseFrame`. `DroneState`
gains `orientation` (truth, owned by the backend) and `pose_estimate` (the latest `DronePoseMsg`
from L1, `None` until one arrives).**

Conventions fixed by this decision, used everywhere ego pose appears:

- Orientation is a unit quaternion `[w, x, y, z]` rotating body vectors into the world frame.
- Uncertainty is the lower-triangular Cholesky factor of the 6×6 pose covariance, row-major,
  in the error order `[δp (3), δθ (3)]`, where `δθ` is a body-frame rotation vector applied on the
  right (`q ⊞ δθ = q ⊗ exp(δθ)`). Shipping the factor, not the covariance, follows the `040`
  amendment: a factor cannot round-trip into a non-PSD matrix, and because it is triangular its
  leading 3×3 block is exactly the factor of the position covariance — the operating picture can
  draw a position ellipsoid from a slice.

## Consequences

- `mission_node` publishes `DronePoseFrame` (truth) and subscribes `/drone_pose_estimates`;
  the simulator scene and the Rerun bridge parse the new envelope.
- L2 planning still bids on truth position. Bidding on the estimate — and on its covariance — is
  the job of `023`.
- `KinematicBackend` never writes an orientation, so `DroneState.orientation` stays identity under
  it; `SmoothBackend` is the backend that gives the camera a direction.
