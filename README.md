# Drone Swarm Autonomy

> **Status: complete and closed (July 2026).** All four layers were built and run end to end
> against the simulator. The project is not under active development — it was deliberately
> concluded to pursue a focused research question that came out of it (see
> [Where this went](#where-this-went)). What is here works, is tested, and is documented; the
> known limitations are listed honestly in [`docs/backlog.md`](docs/backlog.md) rather than
> presented as a roadmap.

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

All four layers are implemented and were run end to end against the simulator.

**L1 — perception**
- **Square-root Unscented Kalman Filter** over a 9-D target state (position, velocity, 3-D extent)
- **Camera and radar observation models** — pinhole projection of an oriented 3-D box; range, azimuth, elevation and Doppler
- **Probabilistic data association** with χ² gating and a null hypothesis, so tracks coast rather than snapping onto clutter
- **Dempster-Shafer classification fusion** with conflict-weighted discounting, which represents *ignorance* explicitly rather than forcing a probability
- **Track lifecycle** — two-point initiation, age-threshold confirmation, coasting, deletion
  (confirmation counts cumulative update cycles rather than a true sliding-window M-of-N —
  one of several simplifications recorded in [`docs/backlog.md`](docs/backlog.md))

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

**Requirements** — Python 3.11+ for the library and tests. The full system additionally needs
Linux or WSL2, ROS 2 Humble, and NVIDIA Isaac Sim.

```bash
uv venv && source .venv/bin/activate
uv pip install -e ".[dev]"
pytest                                     # 48 tests, no simulator needed
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

## What it doesn't do

Stated plainly, because a system that lists only its features is not describing itself:

- **Association is PDA, not joint JPDA.** Exact for one target and for non-overlapping gates,
  which is what the scenes exercise. A joint-event enumerator and a hand-written Hungarian
  solver were built and tested but never integrated — see decision
  [`016`](docs/decisions/016-joint-association.md), including why the work stopped.
- **No tracking metrics.** Accuracy claims here are from live runs against known ground truth,
  not a MOTA/HOTA harness. There is no multi-target benchmark.
- **Fixed-rate fusion.** The filter propagates by a constant `dt`; genuinely asynchronous
  sensors would need a per-measurement timestep.
- **Extent is biased.** Fixed-viewpoint unobservability plus an unscented-transform envelope
  effect; diagnosed, quantified, and accepted rather than fixed.
- **Kinematic drones.** Commanded velocities, not quadrotor dynamics.

The full list, with the reasoning behind each, is in [`docs/backlog.md`](docs/backlog.md).

---

## Where this went

The architecture was built around swappable interfaces for the decision-making layer, with the
intent of replacing what sits behind them — language input, then learned control, then a
vision-language-action model driving the swarm end to end.

Working through that plan produced a sharper question than the platform itself. Classical
coverage control moves a swarm to match an *importance map*, but that map is conventionally
hand-specified. A vision-language model can generate it from what the drones see plus a
natural-language mission — and then the map that guides the swarm is also a map that can be
wrong, shared over an unreliable radio, and corrupted in a way that misdirects the very
observations that would correct it.

Measuring that failure loop is a research contribution. Building a larger platform around it is
not. So this repository was concluded here, and the work continued as a separate, focused
project.

**Continued in:** [**vlm-swarm-coverage**](https://github.com/IshaanBansal2006/vlm-swarm-coverage)
— a simulation rig for decentralized multi-drone coverage driven by a vision-language model.

What carried over: the Isaac↔ROS 2 bridge and its DDS transport profile, the multi-drone scene
and `DroneBackend` seam, the `CoveragePlanner` protocol, the Rerun operating picture, the typed
message schemas, and the Dempster–Shafer fusion from decision
[`015`](docs/decisions/015-ds-conflict-and-decision-rule.md). What did not: the tracking stack,
CBBA, HTN/BT, and the approval gate.

---

## Status

**Complete and closed, July 2026.** Every layer described above was implemented and demonstrated
end to end. No further feature work is planned. Issues and pull requests are welcome but may not
be answered promptly.

Built by [Ishaan Bansal](https://github.com/IshaanBansal2006).
