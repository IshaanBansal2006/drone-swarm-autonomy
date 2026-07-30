# 013 — Observation models, sensing architecture, and target state

**Status:** Accepted
**Date:** 2026-07-27

## Context

L1 needs a measurement model `h(x)` per sensor (passed to `UKF.update`) and a target
state definition. The choice controls what is observable — especially **depth** (range),
which a single monocular camera cannot see directly. This decision also fixes the target
state layout that the UKF, JPDA, and classifier all operate on.

Platform: sensors ride a moving drone (mini-lattice). Camera detections are pixel bounding
boxes `[u, v, w, h]`. A range-capable sensor (radar) is available as the primary depth source.

## Options considered

**Depth observability (how to resolve the camera scale–depth ambiguity `w ∝ S/Z`):**
- **Motion parallax** — observer motion makes bearing/size rates depend separately on depth.
  Free (platform already moves) but slow, motion-dependent, dead at hover.
- **Ground-plane homography** — single camera, but only for on-plane targets (not aircraft).
- **Radar range/doppler** — direct depth + radial velocity. Needs the sensor; Phase 2 in the
  original plan, pulled forward here.
- **Stereo / monocular-depth net** — extra hardware or a learned prior; out of scope for
  classical L1 fusion now.

**Target state / size representation:**
- Point target + class size prior (6-D) — minimal, size assumed not estimated.
- Scalar size (7-D) — one size DOF, no shape aspect.
- Width+height (8-D) — 2-D shape aspect.
- **Full 3-D extent (9-D)** — `Lₓ,L_y,L_z`; most expressive, most unobservable DOF.

**Orientation (needed to project a 3-D extent to a 2-D box):**
- **(a) Yawから velocity heading** — `yaw = atan2(v_y, vₓ)`; no new state, undefined at low speed.
- (b) Orientation-agnostic cross-section — no yaw, but discards shape info.
- (c) Yaw (or full attitude) in the state — correct, but adds a weakly-observable DOF.

## Decision

1. **Depth: radar-primary, motion-parallax fallback.** Radar range/doppler is the primary
   depth + radial-velocity source. If radar is unavailable/fails, the system degrades to
   camera-only and relies on motion parallax (requires platform motion). Graceful degradation.
2. **State: full 3-D extent, 9-D** — `x = [pₓ, p_y, p_z, vₓ, v_y, v_z, Lₓ, L_y, L_z]`.
3. **Orientation: (a) yaw-from-velocity now, with (c) yaw-in-state documented as the upgrade
   path.** Implemented behind a single seam (`_object_yaw`) so migrating to (c) is: change
   that one function to read `x[9]`, bump `state_dim` 9→10, add a yaw row to the motion model,
   and initialize yaw. No other module changes.

**Implementation-fidelity choices (not algorithm-family; revisable):**
- Camera = pinhole `CameraModel` (intrinsics + per-step drone pose as extrinsics).
- Box→image via **8-corner projection** (exact perspective envelope), not a small-object approx.
- Radar = ~~**2-D azimuth** radar `[range, azimuth, doppler]`, no elevation~~ **AMENDED
  2026-07-29 (user decision, resolving backlog D-B5):** radar is **3-D**:
  `[range, azimuth, elevation, doppler]`, matching decision `007`'s `RadarReturn` schema.
  Rationale: sensor redundancy ("more backups" — either sensor alone now recovers 3-D
  position, per the project's graceful-degradation theme) and better vertical behavior (the
  2-D radar left `vz` the weakest-observed component in live thin-slice runs). Consequences:
  radar gate DOF 3→4 (χ²₉₅ = 9.49, same as camera); the "camera supplies elevation"
  complementarity note below is obsolete.

## Consequences

- **`state_dim` becomes 9.** `UKFConfig.state_dim` (currently 6) and `process_noise`
  (currently 6 values) must be updated to 9, with **tiny Q on the extent entries** (size is
  quasi-constant) and a **class-prior initialization** for `Lₓ,L_y,L_z`. *(config.py — next step.)*
- **Multi-sensor from the start.** The tracker (currently single-sensor) must fuse camera +
  radar: per-sensor `h`/`R`, and a fusion order (sequential vs stacked updates). *(tracker.py —
  later step.)*
- **Per-sensor gating.** `JPDAConfig.gate_threshold` (currently `9.21`, χ² 2-DOF) is wrong for
  both sensors. Camera = **4-DOF (χ²≈9.49)**, radar = **3-DOF (χ²≈7.81)**. Gate becomes
  per-sensor. *(config.py — next step.)*
- **Radar-fail → parallax switch** is a new health/observability concern (detect radar loss +
  confirm sufficient platform motion). Likely a small fusion/health module; brushes L4. *(later.)*
- **Orientation coupling (option a):** velocity error rotates the predicted box and biases
  width; the filter may absorb it into `Lₓ,L_y`. Acceptable under (a); removed by (c).
- **Behind-camera corners** are not clipped in the first implementation (assume target in the
  frustum). Real detectors clip; add if partial-visibility cases matter.
- **`h_camera` signature changes** from the stub `h(x)` to `h(x, cam)`; the UKF still receives a
  plain `h(x)` via a per-step closure the tracker builds binding the current camera pose.
