# 017 — Sensor placement: drone-mounted camera, ground-based radar

**Status:** Accepted
**Date:** 2026-09-13
**Amends:** `013` (sensing architecture — radar is ground-based, not drone-borne)

## Context

`013` was written for sensors riding a moving drone. The live harness shipped something else: the
camera is a chase camera held at a fixed offset from the **target** (`chase_camera()` in
`edge/thin_slice_node.py`) and the radar sits at the world origin (`RADAR_POS = 0`). Drone motion
therefore has no effect on anything L1 observes.

`050` reopens the repository for ego-pose estimation. Ego-pose error needs a sensor on an ego
platform to act on; with the harness geometry it has none.

## Options considered

**A. Keep the harness geometry.**
- *Pro:* no work.
- *Con:* ego-pose estimation and any localization-aware behaviour have nothing to act on — fails
  `050`'s contextual-sense gate.

**B. Camera on each drone, radar as a fixed ground station.** ← chosen
- *Pro:* a realistic split — a surveyed ground radar plus airborne electro-optical sensors.
- *Pro:* the radar keeps measuring targets in the world frame independently of any drone's pose,
  so one sensor stays unaffected by ego error.
- *Con:* camera extrinsics now come from an uncertain drone pose; the harness and the scene change.

**C. Both sensors on the drones.**
- *Pro:* every measurement is subject to ego error — the most complete exercise of the estimator.
- *Con:* loses the only sensor at a known position. With no world-frame anchor, the drones and
  everything they observe can drift together undetectably.

## Decision

**Mount the camera on each drone and keep the radar as a fixed ground station at a known position.**

## Consequences

- `CameraModel.R_wc` / `t_w` derive from the drone's pose and the camera mount rather than a
  target-locked offset; `chase_camera()` is retired.
- Radar measurements stay ego-independent. Radar coverage limits now matter, because they decide
  when a target is observed only through drone cameras.
- `013`'s motion-parallax depth fallback becomes live, since the camera's platform actually moves.
- Visibility becomes a modelled quantity: measurement synthesis needs a field-of-view check, since
  what a drone sees now depends on where it is and where it points.
- **Open:** camera mount orientation and field of view. Road signs (`018`) are vertical plates facing
  traffic, so a straight-down camera sees them nearly edge-on.
