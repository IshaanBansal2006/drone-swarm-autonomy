# Backlog — open decisions, known gaps, ideas

Living document. Every open question, deferred decision, known limitation, and idea gets a
line here so nothing lives only in chat history. Decisions get promoted to
`docs/decisions/NNN-*.md` when made; gaps get closed by code; ideas get adopted or struck.

_Last updated: 2026-07-29 (post thin-slice completion)._

---

## Open decisions (user's to make — each needs options+pros/cons before deciding)

| # | Decision | Context | Notes |
|---|---|---|---|
| ~~D-B1~~ | **Config sensor representation — DECIDED 2026-07-29: Option A**, id-keyed dict of a pydantic discriminated union | Consistent with 007's mixed fixed/mounted heterogeneous topology and `Detection.sensor_id` | Implemented in the config.py rewrite. |
| ~~D-B2~~ | **Covariance strategy — DECIDED 2026-07-29: SR-UKF default** (decision `014`) | Bake-off: nominal tie / stress SR-UKF best; structural PSD safety worth +30% cost | `covariance_form="srukf"` default; construct via `edge.filters.make_filter()`; shortcut/joseph retained for comparison. Sigma bug anatomy: `docs/ukf-sigma-point-bug.md`. |
| ~~D-B11~~ | **RESOLVED 2026-07-29: `Q_ext` 1e-8 → 1e-3** (data: `benchmarks/nees_diagnosis.py`) | Block-NEES localized it: pos 2.7 (ideal 3, consistent ✓), extent **9,770**. Sweep: Q_ext 1e-3 → NEES_ext 25, rmse_ext 0.51→0.40. | **Known structural residual (documented, accepted):** extent stays biased ~0.4 m from a *fixed viewpoint* — one extent combination is unobservable along the line of sight, and the corner-envelope `max()` is biased through the UT. `Q_ext` here is honesty/bias compensation, not physical size drift. Future fixes if extent accuracy ever matters: varied viewpoints (orbiting/multi-camera), bias-corrected envelope model. Velocity block is now slightly *over*-conservative (NEES 0.9) — harmless. |
| ~~D-B3~~ | **Zadeh mitigation — DECIDED 2026-07-29: conflict-weighted discounting** (decision `015`) | Reliability semantics fit heterogeneous sensors + defense narrative; keeps associativity | Sub-choice (revisitable, in 015): α from mean pairwise conflict; alternatives = config reliability priors, leave-one-out conflict, time-smoothed α. |
| ~~D-B4~~ | **DS decision rule — DECIDED 2026-07-29: pignistic (BetP)** (decision `015`) | Principled credal→decision bridge; (class, confidence) feeds L3/L4 | Optional unknown-threshold knob not adopted (revisit with L4). |
| ~~D-B5~~ | **Radar elevation — DECIDED 2026-07-29: add elevation** (007 schema wins; 013 amended) | Redundancy ("backups": each sensor alone recovers 3-D position) + fixes weak `vz` | Radar = `[range, az, el, doppler]`, gate DOF 3→4 (χ²₉₅=9.49). |
| D-B6 | **Variable-dt prediction** | `UKF.f` uses fixed `cfg.dt`; real sensors are async | Options: pass dt per predict call vs fixed-rate predict + out-of-sequence handling. Touches config + ukf + tracker. |
| ~~D-B7~~ | **`class_beliefs` type — DECIDED 2026-07-29: (a) DS-native masses** `dict[frozenset[str], float]` | Tracks carry the full evidence/ignorance structure so temporal fusion stays exact; consumers call `decide()`/`pignistic()` for plain numbers | Wired: per-cycle best-associated detections → `combine_discounted` → Dempster into the running belief. Temporal fusion uses plain Dempster + K-guard; discounting the prior is a noted alternative. |
| D-B8 | **Hardware purchase** (future `006-real-hardware.md`) | Pending funds | Analyzed 2026-07-28: Crazyflie 2.1+ ×3 + Flow decks + Crazyradio ≈ $900 (ROS 2 native via crazyswarm2, higher ceiling, needs assembly) vs CoDrone EDU ×3 ≈ $750 (ready-to-fly, closed SDK, free forward ToF). Real-HW L1 = kinematic swarm self-tracking either way (no camera/radar on cheap drones). |
| D-B9 | **Roadmap-slot decisions D2–D5** (design doc) | Timing, venue, category structure, intent-parser tech | User raises when ready; D5 constrained by decision 001 (no local LLM beside Isaac on 8 GB). |

## Known gaps in existing code (implementation work, order flexible)

- ~~`tracker.py` gaps ①②, stale `config.py`, `jpda.py` stub~~ — **ALL CLOSED 2026-07-29**
  (multi-sensor tracker rewrite; see resolved log). Remaining simplifications from that
  rewrite, each deliberate and documented in `tracker.py`/`explanations`:
  - **PDA, not joint JPDA** — exact until gates overlap; joint-event enumeration lands with
    the multi-target scene milestone.
  - **Pseudo-measurement PDA update** — omits the spread-of-innovations covariance inflation;
    optimistic under real clutter.
  - **No angular wrapping** on az/el innovations — gating/likelihoods break for targets near
    the ±π azimuth seam. Fix when scenes wrap around the sensor.
  - **Confirmation is consecutive-age**, not true M-of-N sliding window.
  - ~~Classification fusion unwired~~ — **wired 2026-07-29** (D-B7 resolved; lifecycle test
    asserts fused class + confidence).
- **`h_lidar`:** Phase 3 stub (decision `007` phasing).
- **Housekeeping (found during the 2026-07-30 deep-dive survey):**
  - `tests/edge/test_ukf.py` (user's original, 2026-07-13) coexists with `tests/test_ukf.py`.
    The older one builds 6-D `TrackState`s against a now-9-D `UKFConfig()` default — it passes,
    but it's testing a stale assumption. Merge or retire.
  - `scripts/discovery-server.sh` is **vestigial** — the discovery-server approach was retired
    in favor of the loopback profile (bridge doc Issue 9). Keep only as a documented dead end.
  - Interim `/tracks` JSON should become a real `TrackMsg` publish (finish `040` adoption);
    `mission_node._on_tracks` currently hand-adapts short keys (`id`→`track_id`, `class`→
    `class_label`).
- ~~Thin-slice node runs its own fusion loop~~ — **PORTED 2026-07-30** onto
  `MultiTargetTracker` (683 live cycles: birth→confirm→track, class=vehicle(1.00), err
  ~0.1–0.3 m; interim JSON `/tracks` topic pending 040). Scene truth feed moved from the
  flaky OmniGraph TF branch to direct in-scene rclpy `/target_pose` (bridge doc Issue 11).
  **New D-B11 observation:** under honest plasticity the extent bias expresses as L→~0
  (predicted-envelope inflation compensated by shrinking L) — position/velocity/class
  unaffected; bias-corrected envelope model is the eventual fix.

## Sim / harness ideas (not yet adopted)

- **Bound the target's path** (currently drifts unboundedly; chase camera compensates). Options:
  bounce in a box (adds maneuvers → good future IMM test, breaks pure-CV cleanliness) vs teleport
  loop (innovation spike each lap — stress-tests gating) vs leave unbounded (status quo, fine).
- **Multi-target scene** (2–3 cubes + clutter) — the trigger for implementing JPDA + track birth
  (milestones M3–M4 in the design doc).
- **Replace the measurement synthesizer with Isaac-native sensing:** camera detections via
  Isaac's synthetic-data/annotator pipeline (2-D bboxes are built-in!), custom radar model per
  decision `007` Phase 2. The synthesizer's interface was designed to be swapped.
- **MOTA/MOTP/IDF1 evaluation harness** against Isaac ground truth (M7; also feeds Step 4/5
  baselines per `008`).
- **Publish fused tracks** on a `/tracks` topic (message per `040` sketch) instead of printing —
  the L2/L3 interface seam.
- **Tie scene motion to the playback tick** so GUI Pause works — DONE 2026-07-29 (see below).

## Resolved log (most recent first)

- **2026-07-30 (Steps 1+2 SHIPPED):** `executor.py` (DroneBackend seam + KinematicBackend +
  BT tree factory + per-drone queues), `mission_node.py` (L2 brain: intent→decompose→
  allocate→execute at 10 Hz; replan on new-intent / follow-failure; engagement demo rule),
  `hol/gate.py` (ApprovalGate: pending→approved/denied/auto-denied-timeout, JSONL audit,
  fail-safe "silence never authorizes"; CLI wrapper), `cop/console.py` (one-shot intent CLI,
  blocks on discovery), `cop/rerun_bridge.py` (tracks/fleet/status → Rerun, sim-time
  timeline, --save for headless), fleet prims added to the Isaac scene (renders
  /drone_poses). **LIVE-VERIFIED:** console patrol intent → 2 drones flew their allocated
  polygon legs (probe: DRONES MOVING=True). 34 tests green. Live gotchas logged: rclpy
  Node.executor property collision; one-shot pubs must wait for discovery.
  **Not yet live-tested:** Rerun viewer window (bridge written + imports clean; needs a
  WSLg session), full 5-process demo (Isaac+L1+L2+COP+gate) end-to-end.

- **2026-07-30 (Step 1/2 decisions + coverage):** `022` = **Voronoi** (grid-based, frontier
  seam for unknown-map hardware future — user requirement recorded); `030/031` = **Rerun**
  viewer + Rerun SDK viz transport, ROS 2 stays inter-layer; operator INPUT via CLI console
  first (Rerun hosts no buttons — documented limitation). **Dynamics fidelity ladder** added
  to `008`: kinematic drones now → RL tested on kinematics → switch to real quadrotor
  dynamics/assets → RL re-tested + **training data generated on assets, never kinematics**; seam =
  `DroneBackend` interface in the executor (to be built). `coverage.py` implemented + wired
  into HTN scan; partition property + pipeline tested; 31 green.
  **Next build:** executor (BT leaves → DroneBackend → sim), kinematic drones in the Isaac
  scene + `/drone_cmd` topics, Rerun COP bridge (`pip install rerun-sdk`), CLI operator
  console, replanner triggers.

- **2026-07-30 (Step 1 core):** `autonomy/` implemented per 020/021/040 — `world_state.py`,
  `decomposer.py` (Decomposer Protocol seam + HTN methods goto/track/patrol; scan raises
  pointing at 022; LLMDecomposer stub reserved), `bt.py` (Sequence/Fallback/Action with
  RUNNING memory), `allocator.py` (CBBA: DMG time-discounted scores, best-insertion bundle
  build, fully-connected consensus + bundle truncation — full Choi table deferred to
  realistic comms). 6 new tests incl. full intent→decompose→allocate pipeline; 30 green.
  **Open to proceed:** 022 (scan/coverage), 030/031 (L3), drones-in-Isaac + executor wiring
  (needs sim drones to command), replanner triggers.

- **2026-07-30 (Step 1 kickoff):** L1 node ported onto `MultiTargetTracker` — 683 live
  cycles, class=vehicle(1.00) (see gaps section). Isaac Issue 11 (OmniGraph TF flakiness →
  in-scene rclpy `/target_pose`). **Step-1/2 gates decided by user:** `040` schemas approved
  → implemented (`src/mini_lattice/schemas.py` + round-trip tests); `020` = HTN→BT hybrid
  with an explicit LLM decomposer seam (RL-first roadmap; learned layer will need the slot); `021` =
  CBBA (straight to the deep piece; Hungarian remains an optional eval baseline). All three
  decision docs record every option + rationale. 24 tests green. NEXT: implement `autonomy/`
  (world_state → decomposer/HTN → bt executor → CBBA), one file at a time.

- **2026-07-29 (pt6):** D-B3/B4 decided by user → decision `015` (ALL options + rationale
  recorded). `classification.py` implemented: sensor-aware mass construction, conflict-weighted
  Shafer discounting, guarded Dempster combination, Bel/Pl, pignistic decide. Zadeh paradox
  regression-tested (vanilla exhibits it; pipeline doesn't). 22 tests green. Tracker wiring
  blocked on D-B7 only.
- **2026-07-29 (pt5):** **D-B11 resolved** (`Q_ext` → 1e-3; block-NEES diagnosis in
  `benchmarks/nees_diagnosis.py`; structural extent bias documented). **Multi-sensor tracker
  shipped:** `jpda.py` implemented (gate / PDA betas / combined innovation),
  `measurement_prediction` added to both filters (real S for gating), `tracker.py` rewritten
  (id-keyed sensors, per-step camera poses, two-point radar birth, lifecycle), camera-shim
  removed. Integration tests: full lifecycle (birth→confirm→track→delete) + no-camera-births.
  17 tests green.
- **2026-07-29 (pt4):** D-B2 → **SR-UKF adopted as default** (decision `014` written; factory
  `edge/filters.py` added; thin-slice node switched; 15 tests green). **Deep-dive committed:**
  `docs/ukf-sigma-point-bug.md` — full anatomy of the rows-vs-columns sigma bug (worked 2×2
  example, five reasons it hid, how SR-UKF work exposed it, regression-lock, whiteboard rules).
- **2026-07-29 (pt3):** D-B1 → **option A** implemented (config.py rewritten: id-keyed
  discriminated union, 9-D state, per-sensor χ² gates, process-noise/state-dim validator,
  α=1/κ=1 defaults). D-B5 → **elevation added** to `h_radar` (4-DOF radar; 013 amended).
  **Sigma-point rows-vs-columns Cholesky bug found and fixed** (`ukf.md` second war story;
  regression `test_sigma_points_reproduce_covariance`). Joseph form + **square-root UKF
  implemented** (`srukf.py`, verified equivalent to UKF on identical inputs) and bake-off run
  → data logged at D-B2 (user to pick). New finding logged as D-B11 (NEES/extent
  overconfidence). tracker.py on a single-camera compat shim pending multi-sensor rewrite.
  14 tests green.

- **2026-07-29:** Thin slice complete (scene → bridge → 9-D fusion). Bridge saga documented in
  `docs/isaac-wsl-ros2-bridge.md`. UKF PSD war story in `explanations/edge/ukf.md`. Decision
  renumber 012→013. Decision 001 amended (Foxy→Humble, mirrored networking, loopback profile).
  Scene Pause fix (timeline-gated motion). Offline UKF regression test added (`tests/test_ukf.py`).
- **2026-07-28:** Option B (thin vertical slice first) chosen over config-depth-first. Vision
  charter `008` written (6 steps → learned layer paper). Central topology established for cheap-drone
  hardware demos.
- **2026-07-27:** Decision `013` (radar-primary + parallax-fallback depth, 9-D full-extent state,
  yaw-from-velocity with yaw-state upgrade path). Implementation override recorded in CLAUDE.md.
- **2026-07-13 (user, solo):** Decisions `007` (3 modalities, phased), `010` (UKF), `011` (JPDA),
  `012` (Dempster-Shafer). UKF implemented in `ukf.py`.
