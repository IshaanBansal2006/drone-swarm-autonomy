# Drone Swarm Autonomy

A multi-agent command-and-control platform for drone swarms, built in simulation.

One operator issues high-level intent — *patrol this area*, *track that object* — and the
system fuses heterogeneous sensor data into a single world picture, decomposes the intent
into concrete tasks, allocates those tasks across the fleet, executes them, and gates
sensitive actions on explicit human approval.

Inspired by the architecture of modern defence autonomy platforms.

---

## The four layers

| Layer | Responsibility |
|---|---|
| **L1 · Distributed Edge** | Sensor fusion — noisy detections become tracked objects with position, velocity, size, class and confidence |
| **L2 · Mission Autonomy** | Intent → task decomposition → multi-agent allocation → execution → replanning |
| **L3 · Common Operating Picture** | The operator's live 3-D view of drones, tracks and mission state |
| **L4 · Human-on-the-Loop** | Approve/deny gate for sensitive actions, with a full audit trail |

```
sensors ──▶ L1 fusion ──▶ L2 autonomy ──▶ drones
               │              │
               └──────┬───────┘
                      ▼
              L3 operating picture ──▶ operator
                      ▲                    │
                      └── L4 approval gate ┘
```

---

## What works today

All four layers are implemented and have been run end-to-end against the simulator.

**L1 — perception**
- **Square-root Unscented Kalman Filter** over a 9-D target state (position, velocity, 3-D extent)
- **Camera and radar observation models** — pinhole projection of an oriented 3-D box; range, azimuth, elevation and Doppler
- **Probabilistic data association** with χ² gating and a null hypothesis, so tracks coast rather than snapping onto clutter
- **Dempster-Shafer classification fusion** with conflict-weighted discounting, which represents *ignorance* explicitly rather than forcing a probability
- **Track lifecycle** — two-point initiation, M-of-N confirmation, coasting, deletion

**L2 — autonomy**
- **HTN decomposition** turning intent into primitive, allocatable tasks
- **CBBA** — a decentralised bundle auction with consensus and bundle truncation
- **Voronoi coverage planning** for area scans
- **Behaviour trees** for reactive execution, ticked at 10 Hz

**L3 / L4**
- Live 3-D operating picture with a scrubbable timeline
- Approval gate whose defining property is that **silence never authorises** — an unanswered
  proposal auto-denies on a deadline, and every decision is recorded

Measured on a continuous live run: **683 consecutive fusion cycles**, 0.1–0.3 m position
error at ~400 m range, automatic track birth through confirmation, and classification
converging correctly across two disagreeing sensor types.

---

## Design decisions

Every non-trivial algorithm choice is recorded in [`docs/decisions/`](docs/decisions/) —
context, the options considered, what was chosen, and the consequences. Some worth reading:

- [`010`](docs/decisions/010-state-estimator.md) — why an unscented filter rather than an EKF
- [`013`](docs/decisions/013-observation-and-sensing.md) — how the scale–depth ambiguity shaped the sensor architecture
- [`014`](docs/decisions/014-covariance-strategy.md) — three covariance strategies, benchmarked
- [`015`](docs/decisions/015-ds-conflict-and-decision-rule.md) — Zadeh's paradox and what to do about it
- [`021`](docs/decisions/021-allocator.md) — why CBBA over Hungarian assignment

Two longer write-ups:

- [**The bridge log**](docs/isaac-wsl-ros2-bridge.md) — twelve issues getting a simulator on one
  operating system talking to code on another, including a DDS trap where discovery succeeded
  but no data ever crossed
- [**Anatomy of a silent estimator bug**](docs/ukf-sigma-point-bug.md) — sigma points drawn from
  the wrong triangle of a Cholesky factor: same eigenvalues, same trace, same determinant, and
  completely the wrong covariance

---

## Following the build

The history is organised so you can check out any stage of the project and run it:

```bash
git checkout video-0-bridge       # simulator ↔ ROS 2 bridge only
git checkout video-1-perception   # + the estimation stack
git checkout video-2-autonomy     # + planning, allocation, drones flying
git checkout video-3-full-stack   # + operating picture and approval gate
```

Each tag is self-contained and its test suite passes standalone.

---

## Getting started

**Requirements** — Linux or WSL2, Python 3.10+, ROS 2 Humble, and NVIDIA Isaac Sim for the
simulation itself.

```bash
uv venv && source .venv/bin/activate
uv pip install -e ".[dev]"
pytest                                     # 34 tests, no simulator needed
```

The estimator, allocator and planners are all exercised by the test suite and the benchmarks
in [`benchmarks/`](benchmarks/) without needing the simulator running.

To run the full system, start the simulation scene, then the layer nodes — the sequence is in
[`docs/deep-dive.md`](docs/deep-dive.md).

---

## Layout

```
src/swarm_autonomy/
  schemas.py      cross-layer message definitions
  edge/           L1 — filters, observation models, association, classification, tracking
  autonomy/       L2 — world state, decomposition, coverage, allocation, execution
  cop/            L3 — operating picture and operator console
  hol/            L4 — approval gate
sim/scenes/       simulator scenes
config/           DDS transport profile for the simulator boundary
benchmarks/       covariance strategy comparison, filter consistency sweep
docs/             decisions, engineering write-ups, the full deep dive
```

---

## Where this is going

The classical stack is the foundation, not the destination. The whole architecture is built
around a single swappable interface for the decision-making layer — and the plan is to replace
what sits behind it, in stages:

1. **Language input** — speech → natural-language intent → the same structured goals the system
   already executes today.
2. **Learned control** — a reinforcement-learning policy trained on the same tasks and measured
   by the same metrics as the classical planner.
3. **Vision-language-action** — a model that takes what the drones see *and* what the operator
   says, and produces swarm behaviour end to end, fine-tuned on demonstrations generated in
   simulation.

Every stage plugs into the same interface, runs the same missions, and is scored the same way.
That constraint is the reason the message schemas were frozen before a single layer was
written — without it, none of these approaches could be honestly compared.

Most work on vision-language-action models targets a single robot doing manipulation. Pointing
one at a *swarm*, inside a structured command-and-control system with a human holding veto, is
a different and largely unexplored problem — and that's the part this project is built to
reach.

## Status

Actively developed. Steps 0–2 are complete and running: perception, mission autonomy, the
operator picture and the approval gate. Current work is richer multi-target scenarios,
evaluation metrics, and the language front-end. Open items are tracked in
[`docs/backlog.md`](docs/backlog.md).

Built by [Ishaan Bansal](https://github.com/IshaanBansal2006).
