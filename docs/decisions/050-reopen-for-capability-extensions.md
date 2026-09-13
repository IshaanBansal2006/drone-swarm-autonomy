# 050 — Reopen for capability extensions: SLAM and RL

**Status:** Accepted
**Date:** 2026-09-13
**Amends:** `009` — the repository reopens; `009`'s research pivot stands

## Context

`009` closed this repository to feature work so the research question could move to
`vlm-swarm-coverage`. Its reasoning was specific to research: `008`'s three-brain ablation was
withdrawn because, once the task changed, its deltas would have attributed nothing.

Two capabilities are still worth having in the platform itself, and neither needs a comparative
claim to justify it:

- **Ego-pose estimation.** Every layer treats platform pose as exact ground truth — `deep-dive.md`
  states it directly: *"We assume good onboard nav."* It is the stack's largest standing
  simplification.
- **A learned component** behind one of the seams `008` designed so that brains could be swapped.

CUDA acceleration was considered alongside these and dropped: the current workload has no measured
bottleneck (the 9-D SR-UKF propagates 19 sigma points per cycle).

## Options considered

**A. Keep the repository closed.**
- *Pro:* the finished artifact stays exactly as documented.
- *Con:* the ground-truth-pose assumption stays in place permanently.

**B. Put SLAM into `vlm-swarm-coverage` instead.**
- *Pro:* keeps active work in one project.
- *Con:* enlarges that project rather than giving this platform the capability; RL still has no home.

**C. Reopen as a SLAM + RL research study.**
- *Pro:* nearly all of L2 is reused.
- *Con:* makes the platform a second research project running alongside `vlm-swarm-coverage`, and
  reintroduces the comparative claims `009` withdrew.

**D. Reopen for two sequential, standalone capability extensions.** ← chosen
- *Pro:* each extension removes a stated limitation or fills an existing seam; there is no research
  claim to defend.
- *Pro:* fits the existing tag structure — each extension is a runnable checkpoint.
- *Con:* README's "complete and closed" and `009`'s consequences need rewording; a capability-only
  learned component has to be framed carefully so it is not read as a comparison.

## Decision

**Reopen the repository for two sequential capability extensions — ego-pose estimation (SLAM) and a
learned component (RL) — each shipped under its own tag and each claiming what it does, not that
it outperforms the classical stack.**

Every extension passes the same gate before it is built:

1. **Contextual sense** — it removes an assumption the system states or fills a seam the
   architecture already defines; it is not added for its own sake.
2. **Capability claim only** — the README says what it does and what it does not. No
   "learned beats classical" framing without a controlled comparison.
3. **Behind an interface** — it plugs in behind an existing or newly added Protocol; layers that do
   not consume it do not change.
4. **Simulator-free tests** — like the rest of the suite.

Order, and the variant of each extension, are separate decisions recorded in their own docs.

## Consequences

- `009`'s research pivot stands. `008` steps 3–5, the three-brain ablation and the hardware track
  remain withdrawn; `vlm-swarm-coverage` remains the research venue.
- `009`'s "closed to new feature work" is amended by this doc. The README status line and the
  backlog pivot note need rewording to match.
- Tags continue the existing sequence as `video-4-*` and `video-5-*`, named once the order is set.
- **Precondition found while scoping:** L1's sensors are not mounted on the drones. The radar is a
  fixed ground station (`RADAR_POS = 0`) and the camera is a chase camera held at a fixed offset from
  the *target* (`edge/thin_slice_node.py`). Drone motion therefore has no effect on what L1 observes.
  SLAM depends on resolving this — ego-pose error has no sensor to corrupt until a sensor rides on a
  drone — and so does any learned component coupled to L1. Whether sensors move onto the drones is
  its own decision.
- Adding pose covariance to `DroneState` is a cross-layer schema change and gets its own entry
  alongside `040`.
