# 007 — Sensor modalities

**Status:** Accepted
**Date:** 2026-07-13

## Context

L1 (Distributed Edge) needs to know what sensor data it ingests before any fusion algorithm (010), data association (011), classification fusion (012), or message schema (040) work can begin. The choice also constrains Isaac Sim scene design, L2's allocator (021 — sensor-capability-aware allocation), and the sim-to-real path (006).

## Options considered

**Modalities:**
- **A — Radar only:** single observation model, clean MOT problem. Demonstrates tracking but not multi-modal fusion.
- **B — Radar + EO/IR camera:** classic defense fusion. Two heterogeneous observation models, cross-modal association. Directly maps to Anduril/Lattice.
- **C — Camera + Lidar:** classic AV stack (Waymo/Cruise). Both native in Isaac Sim.
- **D — Radar + EO/IR camera + Lidar:** three modalities. Strongest multi-modal story, highest scope risk. Camera and lidar native in Isaac; radar requires a custom sensor model.

**Mounting topology:**
- **Fixed only:** ground-station sensors, drones are targets/actors. Centralized fusion. Simpler.
- **Drone-mounted only:** each drone is a sensor platform + actor. Decentralized fusion, ego-motion compensation.
- **Both:** ground radar/sensors + drone-mounted sensors. Mixed centralized + decentralized fusion. Most realistic (mirrors actual defense deployments).

**Drone loadout:**
- **Homogeneous:** every drone carries the same sensors. Simpler allocator.
- **Heterogeneous:** different drones carry different sensor packages. Allocator needs capability-awareness.

## Decision

**Option D (radar + EO/IR camera + lidar), both fixed and drone-mounted sensors, heterogeneous drone loadouts.**

Phased implementation order to manage scope:
1. **Phase 1 — Camera.** Fastest to get running in Isaac Sim (native RTX sensor). Establishes the L1 pipeline end-to-end with a single observation model.
2. **Phase 2 — Radar.** Requires a custom sensor model (raycast + range/bearing/doppler noise). Adds the heterogeneous fusion problem (cross-modal association between camera detections and radar returns).
3. **Phase 3 — Lidar.** Native in Isaac Sim. Adds 3D point cloud fusion. Completes the three-modality story.

Each phase produces a working system. The fusion architecture supports all three modalities from day one (schema and interfaces designed for N sensor types), but implementation is incremental.

## Consequences

- **L1 observation models (user writes):** three distinct `h(x)` functions — pixel-space (camera), polar range-bearing-doppler (radar), cartesian point cloud (lidar). Each has its own noise model and coordinate frame.
- **Interview story:** "why three?" — radar gives range in degraded visibility where camera/lidar fail; lidar gives dense 3D geometry camera cannot provide at range; camera gives semantic classification the others cannot. Each covers the others' failure modes.
- **Custom radar sim work.** Before Phase 2, build a radar sensor model in Isaac Sim (OmniGraph node or Python extension: raycast + add range error, bearing error, doppler from target velocity, SNR/clutter). This is sim scaffolding, not core fusion.
- **Mixed topology (fixed + mounted) means L1 must handle both centralized fusion (fixed sensor feeds) and decentralized/distributed fusion (drone-local processing + track sharing).** This is architecturally harder but produces a richer system.
- **Heterogeneous loadouts → allocator (021) must be sensor-capability-aware.** Cannot assign a visual-classification task to a radar-only drone. The `TaskAssignment` schema (040) needs a capability-matching field.
- **VRAM pressure.** Three sensor types rendering on 2–3 drones on 8 GB VRAM is tight. Phasing mitigates this — Phase 1 (camera only) is comfortable; Phase 2–3 may require reducing scene complexity or drone count. Decision 001 already acknowledges this ceiling.
- **Message schemas (040) now scoped.** Raw measurement messages needed:
  - `CameraDetection`: bounding box (u, v, w, h), class_id, confidence, frame_id, timestamp
  - `RadarReturn`: range, bearing, elevation, doppler, snr, timestamp
  - `LidarPointCloud`: point cloud (N × 3 + intensity), sensor_pose, timestamp
  - `Track` (fused output): as sketched in architecture.md, with `source_sensor_ids` referencing contributing sensors
- **Decisions unblocked:** 010 (state estimator), 011 (data association), 012 (classification fusion) can now be decided — the observation model shapes are known. 040 (schemas) can be field-level designed.
- **Decisions constrained:** 021 (allocator) must include capability matching. 005 (intent parser) may reference sensor capabilities ("send a camera-equipped drone to classify target X").
