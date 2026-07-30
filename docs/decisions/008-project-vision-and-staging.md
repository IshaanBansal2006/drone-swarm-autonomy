# 008 — Project vision and staged build plan

**Status:** Accepted
**Date:** 2026-07-28

## Vision

Build a **complete multi-agent command-and-control stack for drone swarms**, from sensor
fusion through mission autonomy to an operator interface with human-on-the-loop safety. An
operator issues high-level intent — *"go into that room, scan it, and track what's inside"* —
and the swarm perceives, plans, allocates and executes autonomously.

**Method:** built incrementally. Each step is a **self-contained sub-project** that runs and
demos on its own *and* is a prerequisite for the next. **Simulation-first (Isaac Sim).**
Hardware is a **later, optional, parallel track** (see Hardware below).

## The staged plan

The system has 4 layers (L1–L4); the build has 6 steps (0–5). They do **not** map 1:1.

| Step | Builds | Sub-project | Standalone deliverable |
|---|---|---|---|
| **0** | L1 | Perception | swarm tracks itself + objects in sim; tracking metrics |
| **1** | L2 | Autonomy (classical) | structured cmd → decompose → allocate → execute |
| **2** | L3 + L4 | COP + safety gate | full stack (classical brain), demo-ready |
| **3** | front of L2 | Language front-end | speech → transcription → structured goals |
| **4** | L2 brain | Learned control policy | a learned baseline on the same tasks/metrics |
| **5** | L2 brain | Learned autonomy layer | end-to-end learned mission control |

### Step detail

- **Step 0 — L1 Perception.** Multi-object tracking + sensor fusion in sim: estimator,
  data association, track lifecycle, classification fusion. Output: a live world picture
  (id, position, velocity, class, confidence) + tracking metrics.
- **Step 1 — L2 Autonomy (classical).** Structured command → task decomposition → multi-agent
  allocation → execution → replanning. Hand-designed brain; no language, no learning yet.
- **Step 2 — L3 + L4.** L3 = 3-D operator terminal (drones, tracks, coverage, mission state,
  real-time). L4 = human-on-the-loop approve/deny gate + audit log + timeout. Full demo.
- **Step 3 — Language front-end.** Speech → transcription → the structured goals L2 already
  understands. This becomes the **symbolic baseline** for later learned approaches.
- **Step 4 — Learned control policy.** A learned baseline trained on the same tasks and
  metrics. Isolates "is learning worth it" and provides a head-to-head comparison point.
- **Step 5 — Learned autonomy layer.** End-to-end learned mission control running inside the
  same architecture, trained on simulator-generated demonstrations.

## Why the later steps are kept separate

Three **interchangeable "brains"** behind one interface enable clean ablation:

- **classical** (Step 1/3) — no learning
- **learned control** (Step 4) — learned policy trained from scratch on these tasks
- **learned autonomy** (Step 5) — learned control with pretrained grounding

Deltas attribute value: **classical → learned** measures the value of learning at all;
**Step 4 → Step 5** measures what pretraining buys. Valid only if all three share the **same
tasks, metrics, and action interface**.

Later choice (defer): task-specific specialists (a performance ceiling) vs. a single
multi-task/goal-conditioned policy (a fairer generalist comparison).

## Cross-cutting principles

- **Swappable-brain interface.** Every candidate brain plugs into one L2 interface — also the
  sim-to-real seam. Lock this abstraction in early, not at Step 5.
- **Every step ships independently.** Anti-scope-creep discipline: never be deep into a long
  build with nothing to demo. Each step should stand alone.
- **Sim-first evaluation.** All quantitative metrics come from Isaac (free ground truth). No
  external positioning budget = no rigorous truth on hardware, so hardware is qualitative.

## Sim dynamics fidelity ladder

Drone fidelity upgrades on a planned schedule, not ad hoc:

1. **Kinematic prims (Steps 1–3):** scripted-motion drones commanded over ROS 2. Fast
   iteration for the classical stack.
2. **Learning tested on kinematics first (Step 4 entry):** cheap rollouts, fast debugging.
3. **Switch to real quadrotor dynamics + assets (before Step 4 exit):** learned components are
   re-tested on the real dynamics, and Step-5 training data is generated on assets, **never on
   kinematics** — the learned stack that matters must not be trained on toy dynamics.

**The seam (scaffolded in the executor):** BT action leaves command a `DroneBackend` interface
(kinematic backend today; quadrotor-style backend later). Only the backend swaps;
decomposer/allocator/BTs/schemas are dynamics-agnostic by construction.

## Hardware (later, optional, parallel track)

- **Start on hardware later in the build**, not now. The critical path is entirely in sim.
- **Steps 0–2 (classical swarm tasks + self-tracking)** can be demoed on cheap camera-less
  drones, commanded centrally from the ground station over ROS 2.
- **The later learned steps cannot run on the cheap drones** — no camera, thin action
  interface. That needs a **camera + onboard compute** platform, decided when reached.
- The concrete drone/hardware purchase decision lives in a future `006-real-hardware.md`.

## Compute & data (for the later steps)

- **8 GB laptop VRAM is under-spec for training** → use **cloud GPU**. Small models may run
  locally for inference; larger ones will not.
- **Drone swarm demonstration datasets don't exist** → **generate in Isaac Sim** (imagery +
  instruction + action trajectories). This is the project's key enabling asset.
- Build on an **open** pretrained model where one fits the task.

## Open items

- **Per-step algorithm decisions** — `010` (estimator), `011` (association), `020`s (L2),
  `030`s (L3), `040` (schemas) — written as each step is reached.
- **Concrete hardware/drone choice** — future `006-real-hardware.md`, pending funds.
