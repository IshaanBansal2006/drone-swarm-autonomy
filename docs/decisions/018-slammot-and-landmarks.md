# 018 — SLAMMOT: prior-mapped road signs, estimated vehicle landmarks

**Status:** Accepted — the sub-decisions listed under Consequences are resolved in `019`
**Date:** 2026-09-13

## Context

`050` adds ego-pose estimation; `017` puts the camera on the drones, so camera measurements now
depend on an uncertain ego pose. Every layer currently treats that pose as exact.

Moving targets alone cannot fix ego pose: shifting a drone and every target it observes by the same
offset leaves every relative measurement unchanged. Stationary landmarks are needed.

## Options considered

**Approach**
- **Inflate measurement noise by projected ego covariance.** Small change, but ego error is never
  estimated or corrected — not SLAM.
- **Off-the-shelf visual SLAM (Isaac ROS).** A demonstrable pose source, but mostly integration, and
  it contends with Isaac Sim for the GPU from inside WSL2.
- **SLAMMOT — ego pose, landmarks and moving targets estimated together.** ← chosen. Reuses the
  SR-UKF (`010`), observation models (`013`), association (`011`) and Dempster–Shafer fusion
  (`012`/`015`). Separating landmarks from moving targets is the defining problem.

**Landmark types considered:** poles, standard-size road signs, lane markings, building walls and
doorways, fiducial markers, surveyed reference points, parked vehicles. Each differs in geometry
(point, line, plane) and prior knowledge (known size, known position, possibly moving).
**Chosen:** road signs and parked vehicles.

**Sign positions**
- **Estimated online.** Full mapping, but no world-frame anchor apart from the radar observing a
  target.
- **Prior map.** ← chosen. Anchors the world frame directly. Building the map is surveying, which
  belongs to a separate project, not this one.

## Decision

**Estimate ego pose, parked-vehicle landmarks and moving targets jointly (SLAMMOT), with
standard-size road signs at known positions from a prior map and parked vehicles of varying size and
colour as estimated landmarks.**

## Consequences

- The scene gains prior-mapped road signs and parked vehicles of several sizes and colours.
- Signs are localization against a map; the *mapping* in SLAMMOT is carried entirely by the vehicles.
- Parked vehicles share the target's `vehicle` class, so landmark-versus-target discrimination is the
  central association problem. Size uses the 3-D extent already in the 9-D state (`013`); colour is a
  categorical attribute through Dempster–Shafer fusion, only as informative as the synthesized
  detector's colour-confusion model.
- Standard sign dimensions give monocular range from apparent size, resolving `013`'s scale–depth
  ambiguity for signs.
- `DroneState` gains a pose covariance — a cross-layer schema change, recorded alongside `040`.
- **Open, each needed before implementation:**
  - one joint filter, or decoupled SLAM and target tracking (Wang, Thorpe et al., IJRR 2007 found the
    decoupled form more tractable);
  - whether the SR-UKF scales with landmark count — cost grows with the cube of the state dimension;
  - whether the prior map is exact or carries map error;
  - vehicle landmark initialization from monocular bearings (delayed, or inverse-depth).
- Ships under tag `video-4-*`.
