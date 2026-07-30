# Mini-Lattice — The Complete Deep Dive

*Everything in this project: the code, the decisions, the math, the bugs, and why each choice
was made. Written 2026-07-30, after Steps 0–2 shipped and ran live.*

---

## How to read this

This is a book, not a reference card. It goes in dependency order — infrastructure, then
perception, then autonomy, then the human layers — because each part is built on the one
before. Read it with the repo open.

| Part | What it covers | Read if you want... |
|---|---|---|
| **0** | The project: vision, staging, why it exists | the elevator pitch and the master plan |
| **1** | Simulation + the ROS 2 bridge | to understand the plumbing everything rides on |
| **2** | L1 Perception (the deepest part) | the estimation math: UKF, SR-UKF, association, DS |
| **3** | L2 Autonomy | planning, behavior trees, multi-agent allocation, coverage |
| **4** | L3 COP + L4 human-on-the-loop | how a person sees and controls the system |
| **5** | The decision ledger — all 16 decisions | why anything is the way it is |
| **6** | Every bug and what it taught | the war stories (best interview material) |
| **7** | Interview prep — what you must derive from memory | the whiteboard checklist |
| **8** | What's next | the road ahead |
| **A** | Appendices: file map, run book, glossary, numbers | quick lookup |

**Notation.** Vectors are lowercase (`x`), matrices uppercase (`P`). `ᵀ` is transpose,
`x̂` an estimate, `n` the state dimension, `m` a measurement dimension. Code references are
`file.py:function`.

---

# Part 0 — The Project

## 0.1 What Mini-Lattice is

A **multi-agent command-and-control platform**: several drones + heterogeneous sensors + one
operator. The operator issues high-level intent ("patrol that area", "track that object");
the system fuses sensor data into a single world picture, decomposes the intent into tasks,
allocates tasks across drones, executes them, and gates sensitive actions on human approval.

It is modeled on **Anduril's Lattice OS** — hence the name — and organized into four layers:

| Layer | Name | Job | Depth (decision `000`) |
|---|---|---|---|
| **L1** | Distributed Edge | sensor fusion → tracked objects | **deep** — interview-defensible |
| **L2** | Mission Autonomy | intent → tasks → allocation → execution | **deep** — interview-defensible |
| **L3** | Common Operating Picture | the operator's live 3-D view | moderate — demo-critical |
| **L4** | Human-on-the-Loop | approve/deny gate + audit | light — safety narrative |

## 0.2 Why it exists

Two reasons, and they pull in the same direction:

1. **Portfolio/interview weapon.** It targets defense-autonomy and hybrid product-research
   roles (Anduril, Shield AI, Skydio, Nvidia GEAR). Two layers go deep enough to defend on a
   whiteboard: perception (L1) and multi-agent planning (L2).
2. **A research vehicle.** The end goal (decision `008`) is a **paper on implementing a
   learned policy in a Lattice structure on swarm drones.**

## 0.3 The staged plan (decision `008`)

Six steps. Each is a self-contained sub-project that **runs and demos on its own** *and* is a
prerequisite for the next. This is the anti-monster discipline: you are never 40% into a paper
with nothing to show.

| Step | Builds | Deliverable | Status |
|---|---|---|---|
| **0** | L1 | swarm tracks itself + objects in sim | ✅ **done + live** |
| **1** | L2 | structured command → decompose → allocate → execute | ✅ **done + live** |
| **2** | L3+L4 | full mini-Lattice, demo-ready | ✅ **done + live** |
| **3** | front of L2 | speech → Whisper → LLM → structured goals | next |
| **4** | L2 brain | custom RL swarm policy (learned baseline) | planned |
| **5** | L2 brain | **learned autonomy layer** → learned autonomy | the goal |

### The baseline ladder (why 3, 4, 5 are separate)

This is the intellectual spine of the whole project. You end with **three interchangeable
brains** behind one interface:

```
                    ┌─────────────────┐
   intent ────────▶ │  L2 brain seam  │ ────────▶ tasks
                    └─────────────────┘
                       ▲     ▲     ▲
              classical│   RL│  learned layer│
             (Steps 1/3)│(Step 4)│(Step 5)
```

- **classical → RL** answers: *is learning worth it at all?*
- **RL → learned layer** answers: *is an expensive pretrained model worth it, or does
  cheap task-specific RL match it?* ← **this is the central comparison**

The comparison is only valid if all three share the **same tasks, same metrics, same action
interface**. That constraint is why the interfaces (decision `040`) were locked before any
layer code was written.

## 0.4 The ground rules

From `CLAUDE.md`, the rules that shape how this repo is built:

- **Every non-trivial decision is the user's.** Options + pros/cons + theory are laid out;
  the user picks; the choice is recorded in `docs/decisions/NNN-*.md` before implementation.
- **Interfaces are stable; implementations are swappable.** Every algorithm sits behind a
  seam so it can be replaced (or degraded to a baseline) without touching its consumers.
- **Everything gets explained twice** — high-level in conversation, line-by-line in
  `explanations/` (gitignored personal study aids).
- **Nothing lives only in chat.** Decisions → `docs/decisions/`; open questions → `docs/backlog.md`;
  engineering war stories → `docs/*.md`.

## 0.5 The physical setup

```
┌──────────────── Windows 11 ────────────────┐   ┌────────── WSL2 (Ubuntu 22.04) ──────────┐
│  Isaac Sim 6.0.1  (C:\IsaacSim)            │   │  ROS 2 Humble                           │
│   - the simulated world + sensors          │◀─▶│  - all mini-lattice code (L1–L4)        │
│   - RTX 4070 Laptop, 8 GB VRAM             │   │  - Python 3.10, numpy/pydantic/rerun    │
│   - internal ROS 2 (jazzy) bridge          │   │  - 4 GB RAM cap                         │
└────────────────────────────────────────────┘   └─────────────────────────────────────────┘
                     └──── Fast DDS over loopback UDP ────┘
```

Why split: Isaac wants the full Windows GPU driver stack; the code wants Linux/ROS 2. The
boundary is crossed with ROS 2 so that "sim → real hardware" later becomes a driver swap
(decision `001`).

---

# Part 1 — The Infrastructure

*The unglamorous part that took the longest and taught the most.*

## 1.1 Why Isaac Sim (decision `001`)

Options were gym-pybullet-drones (fast, no sensor sim), **Isaac Sim** (photoreal camera/radar/
lidar, industry standard at Nvidia/Anduril/Skydio), Gazebo+PX4 (real firmware, dated), or a
custom sim (weeks of work off the critical path).

**Isaac won on sensor realism + industry signal.** The cost: a 2–3 week learning curve and a
hard 8 GB VRAM ceiling that caps scene complexity.

The sub-decision was *where it runs*: **W1 — Isaac on Windows, code in WSL, ROS 2 across the
boundary.** That single choice generated every infrastructure problem in this part, and it was
still correct: the alternative (Isaac inside WSL) means worse drivers, more RAM pressure, and
a version lag.

## 1.2 DDS in one page (you need this to understand everything that broke)

ROS 2 doesn't have a master node. It uses **DDS** (Data Distribution Service), and the
critical mental model is that DDS does **two separate things**:

**1. Discovery** — participants find each other and match publishers to subscribers.
Three mechanisms:
- **Multicast** (default): shout on `239.255.0.1:7400`, everyone hears.
- **Discovery server**: a unicast hub everyone registers with.
- **Initial peers**: each participant unicasts announcements to a list of addresses.

**2. Data transport** — once matched, the actual samples flow. DDS picks a transport *per
matched peer*: **shared memory** if it decides you're on the same host, **UDP** otherwise.
Each participant advertises **locators** — the addresses to reach it on.

> **The key insight, which cost days:** *discovery succeeding does not mean data will flow.*
> You can see a topic in `ros2 topic list` and receive nothing, because matching happened over
> a working channel while data was routed to an unusable transport or an unroutable address.

## 1.3 The bridge saga — 12 issues

Full detail lives in `docs/isaac-wsl-ros2-bridge.md`. Compressed:

| # | Symptom | Root cause | Fix |
|---|---|---|---|
| 1 | — | NAT networking planned | **mirrored networking** (`networkingMode=mirrored`) — WSL shares Windows' IP |
| 2 | `Exec format error` on Windows exes | stale pre-restart state | resolved by the WSL restart; **interop became the debugging workhorse** |
| 3 | topics invisible | **multicast doesn't cross** the WSL boundary | needed unicast discovery |
| 4 | inbound traffic dies silently | **Hyper-V VM firewall** default-blocks inbound | `New-NetFirewallHyperVRule` for UDP 7400–7700 |
| 5 | worked once, then didn't | discovery server (unicast hub) — fragile, and version-mismatched (Fast DDS 2.6 vs 3.x) | eventually **retired** |
| 6 | every `ros2` CLI call crashes `!rclpy.ok()` | `ROS_SUPER_CLIENT=true` corrupts the Humble daemon | never set it; use `--no-daemon` |
| 7 | `rcutils.dll` not found → bridge dead | **`python.bat` doesn't call `setup_ros_env.bat`** (only the GUI launcher does) | `run_scene.bat` wrapper |
| 8 | "Unable to create ROS2 node" | OmniGraph publishers need a **`ROS2Context` node** wired to `inputs:context` | added it |
| 9 | **everything runs, no data crosses** | **the shared-IP trap** (below) | the loopback UDP-only profile |
| 10 | misc | buffered logs, locked files, CLI-vs-rclpy discrepancies | see §6 |
| 11 | `/tf` publishes then goes silent | **OmniGraph disables nodes that error** — the TF branch died silently | publish from the scene's own rclpy |
| 12 | new node undiscoverable | **port-less initial peers only probe participant ids 0–4** | explicit ports 7410–7448 (ids 0–19) |

### Issue 9 in detail — the shared-IP trap

This is the best story in the infrastructure work.

Mirrored networking gives Windows and WSL **the same IP address** (`10.60.166.51`). Fast DDS
saw a peer advertising its own IP and concluded two things, both wrong:

1. *"Same IP ⇒ same host ⇒ use **shared memory** transport."* Windows shared-memory segments
   and Linux shared-memory segments are different universes. Discovery matched; data went into
   a black hole.
2. *"Advertise my locator as `10.60.166.51`."* A WSL process sending to that address loops
   back into WSL. Packets never crossed.

**How it was diagnosed:** by instrumenting *below* the failing layer. A plain Python UDP socket
on `127.0.0.1` sent packets WSL→Windows and Windows→WSL — **both directions worked**. The road
existed; DDS just refused to take it.

**The fix** (`config/fastdds-loopback.xml`, identical on both sides):

```xml
<transport_descriptor>
  <transport_id>UdpLoopbackOnly</transport_id>
  <type>UDPv4</type>
  <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
</transport_descriptor>
...
<rtps>
  <useBuiltinTransports>false</useBuiltinTransports>   <!-- kills shared memory -->
  <userTransports><transport_id>UdpLoopbackOnly</transport_id></userTransports>
  <builtin>
    <initialPeersList> 127.0.0.1 (+ explicit ports 7410..7448) </initialPeersList>
  </builtin>
</rtps>
```

Three things at once:
- **UDP only** → shared memory can never be selected.
- **Whitelist `127.0.0.1`** → every advertised locator is loopback, the address proven to cross.
- **Initial peers** → plain RTPS unicast discovery: no multicast, no discovery server, and
  immune to the Fast DDS 2.6↔3.x version gap.

**Bonus property:** the machine changed networks mid-session (`10.60.x` → `10.0.0.x`) and the
bridge kept working. The profile is network-independent by construction.

## 1.4 The scene (`sim/scenes/thin_slice.py`)

A deliberately minimal, **code-authored** scene — not clicked together in the GUI, because
scripted scenes are reproducible, diffable, version-controlled, and headless-capable (which
Step 5's training data generation will require).

Contents: ground plane, light, one **target cube** moving at constant velocity
`(0.5, 0.2, 0)` m/s, and **two fleet prims** (`d0`, `d1`) that render whatever poses arrive on
`/drone_poses`.

Three structural lessons encoded in it:

1. **`SimulationApp` must be constructed before any Omniverse import** — extensions provide
   those modules, and they load at construction.
2. **Motion is gated on `timeline.is_playing()`** — otherwise the Python loop drives the cube
   regardless of GUI Pause (it did, for days).
3. **Ground truth publishes via the scene's own rclpy**, not OmniGraph (Issue 11). Rule:
   *in-scene rclpy for data the script computes; OmniGraph publishers for data OmniGraph
   computes (clock, sensors).*

## 1.5 The runbook

```bash
# Windows: launch the sim (wrapper sets up ROS 2 env + the DDS profile)
C:\IsaacSim\run_scene.bat C:\IsaacSim\thin_slice.py

# WSL: every shell that talks to Isaac
source scripts/ros-env.sh          # Humble + Fast DDS + the loopback profile
bash scripts/verify-bridge.sh      # checks discovery AND data (the latter is the real test)
```

---

# Part 2 — L1: Perception

*The deepest layer. Everything here is whiteboard material.*

## 2.1 The problem

Sensors give you **fragmented, noisy, ambiguous** observations. L1 turns them into a clean
list of tracked objects with position, velocity, size, class, and confidence.

Three distinct sub-problems, and conflating them is the classic beginner error:

| Problem | Question | Our answer |
|---|---|---|
| **Filtering** | given measurements *of this object*, what is its state? | SR-UKF (`010`, `014`) |
| **Association** | which measurement belongs to which object? | PDA/JPDA (`011`) |
| **Classification fusion** | what *is* it, given disagreeing sensors? | Dempster-Shafer (`012`, `015`) |

Plus a fourth that glues them: **track lifecycle** — when is a new object born, when is it dead?

## 2.2 The state vector

### What a "state" even is

The **minimal set of numbers you must track over time to predict the next measurement.**
Inclusion test: *does this quantity affect a future sensor reading?* Yes → in the state.

### Whose state? (the fork everyone gets wrong at first)

**These are the TARGETS' states, not the drone's.** One state vector + covariance **per
target**. Three targets → three independent 9-D estimates.

Your drone's own pose is a *different concept entirely*: it's a **known input**, feeding the
observation models as the sensor pose.

| | Target state | Ego (drone) state |
|---|---|---|
| Role | **estimated** by the tracker | **known input** |
| Count | one per target | one per platform |
| Appears as | `x` in the filter | `CameraModel.R_wc/t_w`, `h_radar(sensor_pos)` |

*(Caveat: treating ego-pose as perfectly known is an approximation. If drone position is
uncertain, that uncertainty should inflate `R` or be jointly estimated — SLAM-style. We assume
good onboard nav.)*

### The 9-D layout (decision `013`)

```
x = [ pₓ, p_y, p_z,   vₓ, v_y, v_z,   Lₓ, L_y, L_z ]
     └── position ──┘ └── velocity ─┘ └─ 3-D extent ┘
          meters          m/s              meters
```

**Position** — where it is. What radar range and both sensors' bearings measure.

**Velocity** — how it moves. Does **three** jobs, which is what makes it interesting:
1. *Kinematics*: position integrates velocity (`p += v·dt`).
2. *Orientation proxy*: `yaw = atan2(v_y, vₓ)` — the "faces where it moves" assumption. Yaw
   rotates the 3-D box, which changes the **width** the camera sees.
3. *Directly measured by radar Doppler* — rare and valuable.

**Extent** — the target's physical box size. Near-constant (rigid body). Predicts camera box
size, discriminates class, and is the source of the hardest observability problem here.

### How the states couple

Two *kinds* of coupling — keep them separate:

| Coupling | Kind | Mechanism |
|---|---|---|
| position ↔ velocity | **dynamical** (over time) | position integrates velocity → correlated in `P` |
| extent ↔ depth | **measurement** (instantaneous) | camera only ever sees the ratio `size/depth` |
| velocity ↔ horizontal extent | **measurement, option-A only** | velocity → yaw → box width |

The **extent↔depth entanglement** is the scale–depth ambiguity: a big far object and a small
near object produce an identical bounding box. **A monocular camera fundamentally cannot
separate them.** This single fact drove the entire sensing architecture.

### The orientation ladder (9 → 10 → 13)

| dim | Orientation model | Assumption | When |
|---|---|---|---|
| **9** ✅ | yaw = velocity heading | "faces where it moves" | targets move ≈ forward |
| 10 | yaw as an explicit state | upright (roll/pitch ≈ 0) | orientation ≠ motion; need heading out |
| 13 | full attitude **quaternion** | none | tumbling/banking targets |

*Why a quaternion is 4 numbers for 3 rotational DOF:* three Euler angles suffer **gimbal
lock** (a singularity where an axis is lost). A unit quaternion (4 params + unit-norm
constraint) covers the same 3 DOF without it.

**The theory that separates 9-D from 10-D** — and this generalizes far beyond this project:

- **9-D imposes a constraint** (`yaw = heading(v)`), removing a DOF → fewer unknowns, more
  observable, faster convergence — but **biased** exactly when motion ≠ facing (hover, sideslip).
- **10-D relaxes** it → general and unbiased, but **pays in observability** (yaw enters only
  through box width, which also depends on extent — they're entangled) and gains orientation
  as an output.

That's the **constrain-to-observe vs. estimate-everything** tradeoff: bias for variance.

**The migration is cheap by design.** All orientation flows through one function:

```python
def _object_yaw(x):            # observation.py — THE seam
    return atan2(x[4], x[3])   # option A (today)
    # return float(x[9])       # option C — one line, plus state_dim + a motion-model row
```

### "DOF" means three different things here

Mixing these up causes real bugs:

1. **State dimension** — numbers in `x` (9). Sizes the estimation problem.
2. **Physical motion DOF** — a rigid body has 6. Note our 9 states aren't 9 motion DOF: 3
   position + 3 velocity (the 3 *translational* DOF with rates) + 3 *shape* (not motion).
3. **Measurement DOF** — a sensor's measurement dimension → sets the **χ² gate threshold**.
   Camera `[u,v,w,h]` = 4 → χ²₉₅ = 9.49. Radar `[r,az,el,doppler]` = 4 → 9.49. Lidar 3 → 7.81.

Why it matters: **compute** (UKF is `2n+1` sigma points and `O(n³)` Cholesky per step),
**observability** (every added state DOF must be observed by *something* or it drifts — an
unobservable DOF adds a divergence path, not information), and **gating correctness** (the old
config had `9.21`, a 2-DOF value, on 4-DOF measurements).

## 2.3 Observation models (`observation.py`)

`h(x)` answers: *"if the true state were `x`, what would this sensor read?"* These are plain
functions, not classes — stateless maps, correctly so.

### The camera — `h_camera(x, cam)`

State → pixel bounding box `[u, v, w, h]`.

```
1. Build the 3-D box:  8 corners = center ± (Rz(yaw) · half-extents)
2. World → camera:     X_c = R_wc (X_w − t_w)
3. Pinhole project:    u = fx·X_c/Z_c + cx
                       v = fy·Y_c/Z_c + cy
4. Axis-aligned envelope: bbox = min/max of the 8 projected corners
```

Three things to internalize:

- **The division by depth `Z_c` is the nonlinearity.** It is *why* the estimator is a UKF and
  not a linear Kalman filter, and it is *why* size and depth are entangled — both live inside
  that `/Z`.
- **8-corner projection is exact** under perspective and yaw (a turned box projects wider),
  unlike the small-object approximation `w ≈ f·L/Z`.
- **Intrinsics vs extrinsics**: `fx,fy,cx,cy` are fixed calibration; `R_wc, t_w` are the
  camera's pose *right now*. The platform moves, so extrinsics are **per-step inputs** — which
  is why `h_camera` takes `cam` as an argument and the tracker rebuilds it each cycle.

### The radar — `h_radar(x, sensor_pos, sensor_vel)`

State → `[range, azimuth, elevation, doppler]`:

```
rel       = p_target − p_sensor
range     = ‖rel‖                                  ← direct depth
azimuth   = atan2(rel_y, rel_x)                    ← horizontal bearing
elevation = atan2(rel_z, ‖rel_xy‖)                 ← vertical bearing
doppler   = (v_target − v_sensor) · r̂              ← radial velocity, DIRECT
```

- **Range breaks the scale–depth ambiguity.** Once depth is known, the camera's `w ≈ f·L/Z`
  has one unknown left — so **radar makes extent observable too.** You get depth *and* size.
- **Doppler measures velocity directly** — most sensors never do. It tightens `[vₓ,v_y,v_z]`,
  which in option-A also sharpens yaw, which sharpens the predicted camera width. A nice chain.
- **Elevation was added later** (decision `013` amendment): originally 2-D azimuth-only radar,
  with elevation coming from the camera's `v`-axis (elegant complementarity). Changed to 3-D
  for **redundancy** — either sensor alone now recovers 3-D position — and because `vz` was
  measurably the noisiest velocity component without it. Cost: the sensors now overlap more
  and complement less. An honest trade, recorded as such.

### The depth architecture (decision `013`)

The core question: **a monocular camera cannot see depth.** Options considered:

| Option | Mechanism | Verdict |
|---|---|---|
| Motion parallax | observer motion makes bearing/size rates depend on depth | free (platform moves), but slow and **dead at hover** |
| Ground-plane homography | on-plane targets map through a known plane | fails for aircraft |
| **Radar** ✅ | direct range | chosen primary |
| Stereo / mono-depth net | hardware or learned prior | out of scope for classical L1 |

**Decision: radar-primary, motion-parallax fallback** — graceful degradation. If radar fails,
the system reverts to camera-only, where parallax keeps `(Z, L)` weakly observable *as long as
the platform moves.*

## 2.4 The Unscented Kalman Filter (`ukf.py`)

### Why UKF and not EKF

Both handle nonlinear `h`. The EKF linearizes with hand-derived **Jacobians**; the UKF pushes
a deterministic point cloud through the true nonlinear function.

Decision `010`'s reasoning: **three heterogeneous sensors** means three separate Jacobian
matrices to derive and maintain with the EKF. With the UKF, adding a sensor is *writing one
function `h(x)`*. Plus the perspective divide and polar radar are nonlinear enough that
first-order linearization degrades.

### The unscented transform

**The idea:** it's easier to approximate a *distribution* than an arbitrary *function*. Pick
`2n+1` deterministic points that exactly encode a Gaussian's mean and covariance, push them
through the true nonlinearity, and refit a Gaussian to the transformed points.

Accuracy: **2nd order for any nonlinearity** (3rd for Gaussians), vs the EKF's 1st-order
truncation. No Jacobians anywhere.

### The weights

```
λ  = α²(n + κ) − n                        ← scaling parameter

Wm[0] = λ / (n + λ)                       ← mean weight, center point
Wc[0] = λ / (n + λ) + (1 − α² + β)        ← covariance weight, center point
Wm[i] = Wc[i] = 1 / (2(n + λ))            ← all 2n others
```

- **α** controls spread (how far points sit from the mean).
- **β** encodes prior distribution knowledge; **β = 2 is optimal for Gaussians** (it corrects
  the 4th moment / kurtosis).
- **κ** is a secondary knob.

> ⚠️ **The n=9 trap.** The textbook default `α=1e-3, κ=0` gives `λ = 1e-6·9 − 9 ≈ −9`, so
> `n+λ ≈ 9e-6` and the center weights explode to **±10⁶**. Negative-weight covariance sums are
> not PSD-guaranteed *even in exact arithmetic*. **We use `α=1, κ=1 → λ=1 → all 19 weights
> positive**, making reconstructed covariances PSD by construction.
> **Lesson: sigma-point tuning is dimension-dependent. Defaults that work at n=3 are hostile at n=9.**

### Sigma points

```
χ₀     = x
χᵢ     = x + [√((n+λ)P)]ᵢ      i = 1..n
χ_{n+i} = x − [√((n+λ)P)]ᵢ
```

`√P` is a matrix square root — the Cholesky factor. **Which vectors you take from it is
subtle enough that it was a real bug for 16 days** (§2.5).

### Predict

```
χᵢ ← f(χᵢ)                                    push through motion model
x⁻ = Σᵢ Wm[i]·χᵢ                              weighted mean
P⁻ = Σᵢ Wc[i]·(χᵢ − x⁻)(χᵢ − x⁻)ᵀ  +  Q       weighted spread + process noise
```

`f` is **constant velocity**: position += velocity·dt, velocity unchanged, extent unchanged.
CV assumes acceleration is zero-mean noise absorbed by `Q`. (Maneuvering targets are what
**IMM** — a bank of models — exists for; decision `010` keeps it as the documented upgrade.)

### Update

```
Zᵢ      = h(χᵢ)                                       push through observation model
z_pred  = Σ Wm[i]·Zᵢ                                   expected measurement
S       = Σ Wc[i]·(Zᵢ−z_pred)(Zᵢ−z_pred)ᵀ + R          innovation covariance
Pxz     = Σ Wc[i]·(χᵢ−x)(Zᵢ−z_pred)ᵀ                   cross-covariance
K       = Pxz · S⁻¹                                    Kalman gain
ν       = z − z_pred                                   innovation ("the surprise")
x⁺      = x + K·ν
P⁺      = P − K·S·Kᵀ
```

**`Pxz` is the star.** It replaces the EKF's measurement Jacobian `H` — the state↔measurement
relationship is *measured empirically from the sigma points* rather than differentiated.

`update()` returns `(state, ν, S)` deliberately: innovation and `S` are exactly what
association needs downstream (Mahalanobis distance, likelihoods).

### Sequential vs stacked updates

Two sensors per cycle can be fused by stacking measurements into one big vector, or applied
**sequentially**. We do sequential, **radar first**:

> For a *linear* `h`, update ordering is mathematically irrelevant. For a *nonlinear* `h`, it
> matters: the milder nonlinearity (radar) tightens `P` first, so the harsher one (camera's
> perspective + yaw) linearizes around a better-conditioned prior.

## 2.5 The sigma-point bug — a full anatomy

*Found 2026-07-29 while implementing the SR-UKF. Lived in the code since day one. Full
write-up: `docs/ukf-sigma-point-bug.md`. This is the single best interview story in the repo.*

### What the code did

```python
sqrt_P = np.linalg.cholesky((n + lam) * P)   # lower-triangular L, L @ L.T = P
sigma[i + 1] = x + sqrt_P[i]                 # ROW i of L   ← the bug
```

### What the math requires

Sigma points must **reconstruct the covariance exactly**: `Σᵢ Wᵢ(χᵢ−x̄)(χᵢ−x̄)ᵀ = P`.

With offsets `sᵢ` drawn from a matrix `M`, that sum is `Σ sᵢsᵢᵀ`, and:

- offsets = **columns** of `M` → `Σ sᵢsᵢᵀ = M Mᵀ`
- offsets = **rows** of `M` → `Σ sᵢsᵢᵀ = Mᵀ M`

`np.linalg.cholesky` returns **lower**-triangular `L` with `L Lᵀ = P`. So **columns are
correct; rows reconstruct `LᵀL ≠ P`.**

### The 2×2 that makes it concrete

Take `P = [[4, 2], [2, 2]]`, so `L = [[2, 0], [1, 1]]` (check: `LLᵀ = P` ✓).

- **Columns** `(2,1)` and `(0,1)`: `Σ ccᵀ = [[4,2],[2,2]]` = **P** ✓
- **Rows** `(2,0)` and `(1,1)`: `Σ rrᵀ = [[5,1],[1,1]]` ✗

Same trace (6), same determinant (4), **same eigenvalues** — a different matrix. The sigma
cloud had the right *size* by every invariant you'd casually check, and the wrong *shape*:
rotated correlation directions. **Every predict and update ran on a rotated belief.**

### Why it hid for 16 days — five reasons

1. **No checked invariant was violated.** `LᵀL` is symmetric, positive-definite, right scale.
   Nothing crashed; tracks converged.
2. **The filter is self-consistent in the error** — predict reconstructs covariance from the
   same mis-shaped cloud, so each cycle is internally coherent.
3. **Weak nonlinearity forgives.** Errors hid inside measurement noise.
4. **The symptom was misattributed.** When the camera's harsh nonlinearity interacted badly
   with the mis-shaped spread, `P` went indefinite — which *looks* like the famous
   `P − KSKᵀ` fragility, so guards were added there instead. (Post-fix, PSD repairs dropped to
   **zero across all benchmark scenarios including stress** — strong evidence the rows/cols bug
   was a primary driver of the instability.)
5. **Precedent bias.** "Spread along rows of the Cholesky result" is a real pattern in
   well-known code (FilterPy) — but with **scipy**, whose default factor is **upper**
   triangular (`RᵀR = P`), where rows *are* correct. Porting the idiom to numpy's lower factor
   silently inverts it.

### How it was found and fixed

Implementing the **square-root UKF** forced explicit reasoning about factor conventions
(SR-UKF's entire state *is* the factor). Auditing the standard UKF against that answer exposed
the row indexing.

```python
L = np.linalg.cholesky(scaled)
sqrt_P = L.T            # rows of Lᵀ == columns of L
```

Locked by `test_sigma_points_reproduce_covariance`, which rebuilds `P` from the weighted sigma
deviations on a **strongly correlated** random SPD matrix. *(Test-design lesson: for diagonal
`P`, `LᵀL = LLᵀ` and the bug is invisible. Test data must exercise the failure mode.)*

### The rules to keep

1. For factor `M` with `MMᵀ = P`, sigma offsets are the **columns** of `M`.
2. **numpy** → lower `L` (use columns); **scipy** → upper `R` (rows work). Name the convention
   before writing the indexing.
3. Symmetric-PSD-and-right-scale ≠ right. Test **reconstruction**, not health.
4. **A "numerical stability" symptom can be a correctness bug in disguise.** When guards keep
   firing, audit the math upstream of the guard.
5. Re-deriving a component from a different formulation is a powerful audit of the original.

## 2.6 Covariance strategies and the SR-UKF (decision `014`)

### The problem

`P⁺ = P − K S Kᵀ` — the "shortcut" form — is not PSD-guaranteed under floating-point error and
strong nonlinearity. It bit twice: a live crash, and a test-suite counterexample where stored
`P` had eigenvalue −0.03.

### Three strategies, benchmarked

| Strategy | Mechanism | PSD safety |
|---|---|---|
| **shortcut** | `P − KSKᵀ`, symmetrize, eigenvalue-floor on violation | reactive repair |
| **Joseph-analog** | `P − K·Pxzᵀ − Pxz·Kᵀ + K·S·Kᵀ` | quadratic in `K` → first-order insensitive to gain error |
| **SR-UKF** ✅ | carry the Cholesky **factor** `S` (`P = SSᵀ`) | **structural** — `SSᵀ ⪰ 0` always |

*Note: the linear-KF Joseph form `(I−KH)P(I−KH)ᵀ + KRKᵀ` has no direct UKF analog because no
`H` exists. The `Pxz`-based expression above is the standard equivalent.*

### The bake-off (`benchmarks/covariance_strategies.py`, 20 seeds × 300 steps)

| scenario | strategy | repairs | rmse_pos | rmse_vel | NEES | µs/step |
|---|---|---|---|---|---|---|
| nominal | all three | 0 | 0.056 | 0.273 | ~9784 | 1690 / 1640 / 2115 |
| stress | shortcut | 0 | 0.591 | 0.539 | 115961 | 1710 |
| stress | joseph | 0 | 0.460 | 0.440 | 115594 | 1743 |
| stress | **srukf** | 0 | **0.440** | 0.450 | **95010** | 2243 |

**Reading it honestly:** post-sigma-fix, repairs are zero everywhere — the emergency was the
bug, not the update form. The strategies then differ only at the margins: tie under nominal,
SR-UKF best under stress, Joseph free-second, SR-UKF ~30% slower.

**Decision: SR-UKF as the default.** Structural safety + best stress numbers, and 2.2 ms/step
at n=9 is nowhere near any budget. Shortcut and Joseph stay selectable via
`UKFConfig.covariance_form` for comparison and teaching.

### How the SR-UKF works (`srukf.py`)

Instead of `P`, carry lower-triangular `S` with `P = SSᵀ`. Three primitives replace the
covariance arithmetic:

- **QR decomposition** — recompose a factor from weighted deviation columns:
  `colsᵀ = QR ⇒ cols·colsᵀ = RᵀR`, so `S = Rᵀ`.
- **Rank-1 Cholesky update/downdate** — fold in the center sigma point (`Wc[0]` may be
  negative → downdate) and apply the measurement decrease, in `O(n²)` via hyperbolic rotations.
- **Eigenvalue-floor rebuild** — the repair path, only reachable if a **downdate** fails (the
  SR-UKF's one residual failure mode), counted in `self.repairs` for fair comparison.

Sigma points need **no per-step Cholesky** — the factor *is* the input (offsets are `γ ·
columns of S`). Working in the factor also squares the usable numerical dynamic range.

`test_srukf_matches_ukf_on_linear_case` pins SR-UKF ≡ UKF (means to 1e-6, covariance via
`SSᵀ`) — same math, different carrier, proven equal.

## 2.7 Filter consistency — NEES and the `Q_ext` story (D-B11)

### The metric

**NEES** (Normalized Estimation Error Squared):

```
ε = (x − x̂)ᵀ · P⁻¹ · (x − x̂)          E[ε] = n  for a consistent filter
```

Interpretation — this is the filter's **honesty check**:
- `ε ≈ n` → the filter's claimed uncertainty matches its actual error. ✅
- `ε ≫ n` → **overconfident**: small `P`, large errors. The dangerous direction (downstream
  gating rejects real detections; the operator trusts a lie).
- `ε ≪ n` → over-conservative: wasteful but safe.

*(Its measurement-space sibling is **NIS**, computed from `νᵀS⁻¹ν` — usable without ground
truth, which matters on real hardware.)*

### The diagnosis

Aggregate NEES was **~9,800 vs an ideal 9.** Splitting it per block localized it instantly:

| `Q_ext` | NEES_pos | NEES_vel | NEES_ext | rmse_ext | rmse_pos |
|---|---|---|---|---|---|
| 1e-8 | 2.72 | 10.35 | **9770.3** | 0.506 | 0.056 |
| 1e-6 | 2.38 | 3.14 | 2707.4 | 0.491 | 0.055 |
| 1e-5 | 2.25 | 0.91 | 804.9 | 0.489 | 0.054 |
| 1e-4 | 2.25 | 0.91 | 172.4 | 0.477 | 0.054 |
| **1e-3** ✅ | 2.18 | 0.90 | **25.0** | **0.402** | 0.054 |

**Position was consistent all along** (2.72 vs ideal 3.0). Extent was the entire problem.

**The mechanism — the "smug filter":** `Q_ext = 1e-8` says *"size never changes"*, which is
physically true for a rigid body. So the filter drives extent covariance toward zero. But the
extent *estimate* is biased (below), so you get a tiny claimed variance dividing a not-tiny
error → NEES explodes. The filter became certain of something wrong.

**The fix:** `Q_ext = 1e-3`. NEES_ext 9,770 → 25, and RMSE *improved* (0.51 → 0.40) because a
plastic estimate can still move toward truth.

### The residual, honestly

Extent stays biased ~0.4 m and NEES_ext = 25 is still above 3. This is **structural, not
tunable**:

1. From a **fixed viewpoint**, one extent combination is unobservable along the line of sight.
2. The corner-envelope `max()` is a **biased estimator** through the unscented transform:
   `E[max(corners)] ≠ max(E[corners])` — averaging over sigma points systematically inflates
   the predicted box, which the filter compensates by *shrinking* `L`. (Live runs later showed
   extent collapsing toward ~0 under honest plasticity — exactly this effect.)

`Q_ext` here is **bias/honesty compensation, not physical size drift** — an important
distinction to state out loud. Real fixes if extent accuracy ever matters: varied viewpoints
(orbiting/multi-camera), or a bias-corrected envelope model.

> **The generalizable lesson:** process noise is not only "how much the world moves." It is
> also how much you admit your *model* is wrong. Tuning `Q` against NEES is how you make a
> filter honest.

## 2.8 Data association (`jpda.py`, decision `011`)

### The problem

`N` tracks, `M` measurements, no labels. Which measurement belongs to which track? Which are
new targets? Which are clutter?

| Approach | Idea | Trade |
|---|---|---|
| **GNN** (Hungarian) | optimal hard 1-1 assignment | simple; one wrong assignment corrupts a track |
| **JPDA** ✅ | soft: weight *every* gated measurement by probability | robust to clutter; memoryless; merges identities on crossings |
| **MHT** | keep a hypothesis tree over time, defer decisions | gold standard; most complex; needs pruning |
| Learned | transformer/GNN cost matrices | needs data; black box |

**Decision: JPDA, with MHT as the documented fallback** if crossings/clutter expose its known
failure modes. The upgrade is an interface swap — filter and observation models don't change.

### Step 1 — Gating

```
d² = (z − z_pred)ᵀ · S⁻¹ · (z − z_pred)   <   χ²(0.95, m)
```

The **Mahalanobis distance** is covariance-aware: the acceptance ellipsoid stretches along
directions where `S` says you're uncertain. It's a cheap filter that shrinks the association
problem before the expensive part.

Two details that were bugs before they were features:
- The threshold is **χ² in the measurement dimension** — per-sensor, from config. (The old
  single `9.21` was a 2-DOF value applied to 4-DOF measurements.)
- `S` must be the **real unscented innovation covariance**, obtained from
  `filter.measurement_prediction(state, h, R)`. The original code returned `R.copy()` — sensor
  noise only — so gates ignored state uncertainty entirely. Now gates are wide for new tracks
  and tight for converged ones, which is the whole point.

### Step 2 — Association probabilities (the β weights)

```
β_j  ∝  P_D · N(z_j ; z_pred, S)          "detected AND looks like the prediction"
β_0  ∝  λ_fa · (1 − P_D · P_G)            "missed, or fell outside the gate" (null hypothesis)
      then normalize so Σβ = 1
```

`β_0` is the **null hypothesis** — the probability that *none* of the gated detections belongs
to this track. When a big innovation makes every likelihood small, mass flows to `β_0` and the
track **coasts** on its prediction instead of snapping to clutter. That single mechanism is
why soft association beats hard.

With tiny clutter density and one clean detection, `β ≈ [0, 1]` — soft association gracefully
degrades to hard.

### Step 3 — Combined innovation

```
ν_combined = Σⱼ βⱼ · (zⱼ − z_pred)
```

The *expected* innovation under the association posterior. The null hypothesis contributes
zero, so the correction automatically shrinks when the target probably wasn't seen.

### Two documented simplifications

1. **This is PDA, not joint JPDA.** Exact for a single target or non-overlapping gates. Full
   JPDA enumerates *joint* association events so competing tracks share probability mass —
   that layer arrives with multi-target scenes; interfaces don't change.
2. **Pseudo-measurement update.** The tracker feeds `z_pred + ν_combined` to the filter as if
   it were a measurement. This omits PDA's covariance-inflation ("spread of the means") term,
   making the filter **optimistic under real clutter**. Know this — an interviewer may probe it.

## 2.9 Track lifecycle (`tracker.py`)

Association handles *existing* tracks. Something must create and destroy them.

**Birth — two-point differencing.** An unmatched **radar** detection is inverted to a world
position (spherical → cartesian) and paired with one from the *immediately previous* cycle. If
the implied speed `‖Δp‖/dt ≤ 15 m/s` (a plausibility gate), a track is born with position and
**finite-difference velocity**.

- *Radar-only* because a camera bbox has no depth — you cannot invert it to a position. This
  is pinned by `test_tracker_no_false_births_from_camera`.
- *Never zero-velocity*, for a reason discovered the hard way (§6): velocity → yaw → box width,
  so a zero-velocity prior makes sigma points span *every heading*, and the camera update
  overshoots into divergence.

**Confirmation** — a track is emitted only once `age ≥ confirm_hits` (3). Tentative tracks are
maintained internally but never published: clutter suppression.

**Death** — `misses ≥ max_misses` (5) consecutive cycles without a real association. Between
those, the track **coasts** on prediction, which is what carries it through occlusions.

**Confirmation and deletion thresholds are the knobs that trade track continuity against
false-track rejection.** Tuning them is a core tracking skill; the right values depend on
`p_detection` and clutter density.

## 2.10 Classification fusion — Dempster-Shafer (`classification.py`, decisions `012` + `015`)

### Why not Bayes

Sensors have different classification *vocabularies*: a camera sees fine classes, a radar sees
"ground mover vs air", lidar sees shape. **A Bayesian classifier cannot say "I don't know"** —
it must spread probability over classes, and a uniform posterior is indistinguishable from
ignorance. Dempster-Shafer represents ignorance *explicitly*, as mass on the whole frame Θ.

### The objects

- **Frame of discernment** Θ = `{vehicle, person, aircraft, unknown}`.
- **Mass function** `m`: assigns belief to **subsets** (focal elements), summing to 1. Mass on
  `{vehicle, person}` means "a ground mover, I can't say which."
  *(Implementation note: focal sets are `frozenset` keys — sets aren't hashable, frozensets are.)*
- **Belief** `Bel(A) = Σ_{B ⊆ A} m(B)` — committed support, the **lower** bound.
- **Plausibility** `Pl(A) = Σ_{B ∩ A ≠ ∅} m(B)` — not-ruled-out, the **upper** bound.
- The interval `[Bel, Pl]` *is* the uncertainty — the thing a probability cannot express.

Mass construction here: camera/lidar commit `confidence` to a singleton; radar commits to its
broad category; leftovers go to **Θ, never to the class `"unknown"`** — "could be anything" and
"is the unknown type" are different objects.

### Dempster's rule and Zadeh's paradox

```
m(A) = (1/(1−K)) · Σ_{B∩C=A} m₁(B)·m₂(C)        K = Σ_{B∩C=∅} m₁(B)·m₂(C)
```

Mass flows to **intersections**; mass landing on ∅ is **conflict** `K`, divided out.

**Zadeh's paradox:** camera says `{truck}: 0.99`, IR says `{person}: 0.99`, both leave 0.01 on
`{car}`. Almost every intersection is empty (`K ≈ 0.9999`), and the tiny surviving `{car}`
mass, renormalized, yields **`car` with certainty ≈ 1.0** — a class *neither sensor supported*.

This is the most-asked DS interview question. Mitigations considered (decision `015`):

| Option | Mechanism | Semantics |
|---|---|---|
| **Yager's rule** | conflict mass → Θ instead of renormalizing | "disagreement means we don't know" |
| **Conflict-weighted discounting** ✅ | discount each source toward Θ by how much it conflicts, *then* Dempster | "disagreement means someone is unreliable" |
| Murphy's average-then-combine | average masses, then self-combine | dilute outliers |

**Decision: conflict-weighted discounting** — it matches the physical reality (a jammed radar
or glare-blinded camera *should* lose voting power), keeps Dempster's associativity after
discounting, and gives the strongest defense-autonomy narrative (sensor health → trust).

```
α_i = 1 − mean pairwise conflict of source i with the others     ← reliability
Shafer discounting:  m^α(A) = α·m(A) for A ≠ Θ
                     m^α(Θ) = 1 − α·(1 − m(Θ))
```

α=1 leaves a source untouched; α=0 silences it entirely. Regression-tested: **vanilla Dempster
provably exhibits the paradox; the discounted pipeline keeps >50% mass on ignorance and
refuses a confident call.** A guard also catches `K → 1` at the combiner and returns the
vacuous mass with a logged warning — never emit paradoxical certainty silently.

### The decision rule (D-B4)

You hold intervals; you must emit a label.

| Rule | Character | Weakness |
|---|---|---|
| Max-plausibility | optimistic ("least ruled out") | can pick a class with almost no committed support |
| Max-belief | conservative ("only proven support") | freezes under ignorance (everything ties at 0) |
| **Pignistic (BetP)** ✅ | the principled middle | flattens the interval at decision time |

```
BetP(c) = Σ_{A ∋ c} m(A) / |A|            split each focal set's mass uniformly over its members
```

Smets' **transferable belief model**: reason at the *credal* level (masses, intervals), act at
the *pignistic* level. The uniform split is the insufficient-reason principle applied inside
each focal set. Chosen because the emitted `(class, confidence)` is exactly what L3 display and
L4 gating consume. Test-pinned invariant: **Bel ≤ BetP ≤ Pl**, always.

### Where it lives on a track (D-B7)

`Track.class_beliefs` stores the **DS-native mass function**, not a flattened per-class
probability. Rationale: masses carry the full ignorance structure, so *next* cycle's evidence
fuses exactly. Storing BetP would store the **decision** instead of the **evidence** — and you
cannot correctly fuse new evidence into an already-flattened distribution. That would quietly
undo the entire reason for choosing DS.

Per cycle: each sensor's most-probable associated detection contributes a mass function →
`combine_discounted` across sensors → Dempster-combine into the track's running belief.

## 2.11 How L1 runs live

`edge/thin_slice_node.py` — the harness that connects the sim to the stack:

```
Isaac /target_pose (ground truth)
  → finite-difference velocity
  → SYNTHESIZE camera [u,v,w,h] and radar [r,az,el,doppler] via the REAL h() + Gaussian noise
  → Detection messages (with class labels)
  → MultiTargetTracker.step()  ← the real decided stack
  → /tracks (interim JSON)
```

**Why synthesize measurements instead of using Isaac's cameras?** This is the standard
estimation-development workflow: before real sensors exist, validate the filter against
measurements generated from truth *through the real `h` functions* plus noise. It also *tests
the models* — a bug in the 8-corner projection surfaces here, not months later. The
synthesizer is a **test harness with a swappable interface**: real Isaac sensors + a detector
replace it on the same `Detection` boundary, and nothing downstream changes.

**Live result:** 683 consecutive fusion cycles, position error 0.1–0.3 m, class `vehicle(1.00)`
by DS fusion, automatic birth → confirmation → sustained track.

---

# Part 3 — L2: Mission Autonomy

*From "patrol that area" to drones actually flying.*

## 3.0 The pipeline

```
StructuredIntent          "patrol this polygon"
      │
      ▼  HTN decomposer (020)  [+ Voronoi coverage (022) for scan]
   [Task, Task, Task]          primitive, allocatable, drone-agnostic
      │
      ▼  CBBA allocator (021)
   {d0: [T1, T3], d1: [T2]}    who does what, in what order
      │
      ▼  Executor → Behavior Trees (020) → DroneBackend
   drones move
```

Each arrow is a **seam** — an interface with a swappable implementation. That is deliberate:
the decomposer seam is where the Step-3 LLM and Step-5 learned layer plug in; the backend seam is where
real quadrotor dynamics plug in.

## 3.1 Task decomposition — HTN (`decomposer.py`, decision `020`)

### The options

| Option | Theory | Verdict |
|---|---|---|
| **HTN** ✅ | recursive method decomposition of abstract tasks | classic AI planning, explainable, whiteboard-defensible; rigid (you author every recipe) |
| **Behavior Trees** ✅ (execution) | reactive tick-based composition | robotics-standard, composable; not a *planner* |
| LLM decomposition | a model emits the task graph | flexible; where Step 5 lives; nondeterministic, premature now |
| PDDL + planner | formal search | provably correct; heavyweight for 3 mission types |

**Decision: hybrid — HTN plans, BTs execute, with an explicit LLM seam.**

The reasoning is worth restating because it's strategic, not just technical:
1. **The LLM slot is a certainty, not a maybe** (Steps 3 and 5 need it). Designing the
   decomposer as a swappable component *now* means the classical HTN later becomes a
   **baseline** rather than something ripped out — which is exactly what the project's
   comparison ladder requires.
2. **Roadmap order is RL-first**, so Step 1 stays fully classical.
3. **Each formalism carries a different story**: HTN = the *planning* interview answer
   (methods, preconditions, decomposition); BT = the *execution* answer (reactivity, recovery).

### How the HTN works

A **method library** keyed by verb. `decompose(intent, world)` dispatches to `_method_<verb>`;
each method expands an abstract intent into **primitive Tasks**.

```python
class Decomposer(Protocol):                          # ← THE seam
    def decompose(self, intent, world) -> list[Task]: ...

class HTNDecomposer:    ...  # today
class LLMDecomposer:    ...  # reserved stub — Step 3/5
```

*(A `Protocol` gives structural typing: implementations need no inheritance, just the method.)*

The three implemented methods:

- **`goto`** → one waypoint task.
- **`track`** → a `follow_track` task with `required_capability="camera"` (decision `007`'s
  capability-awareness made real) and `reward=2.0` — tracking outranks patrolling in the
  allocator's scoring.
- **`patrol`** → polygon boundary → per-edge legs, **grouped by fleet size**. That grouping is
  textbook HTN behavior: *methods branch on world state*. Note it's a **perimeter circuit**,
  not area coverage — those are different problems.
- **`scan`** → delegates to the injected coverage planner (§3.4). Before decision `022` existed
  it raised `NotImplementedError` **pointing at the open decision** — writing a lawnmower sweep
  there would have silently decided the fork.

Errors are actionable by rule: an unknown track id lists the known ids; a missing field names
the field.

## 3.2 Behavior trees (`bt.py`)

### The model

A BT is a **function evaluated repeatedly**. Every `tick` returns `SUCCESS`, `FAILURE`, or
`RUNNING`.

**`RUNNING` is the whole point.** The tree yields control every tick, so the layer above can
re-tick at its own rate, preempt, or rebuild it as the world changes. Contrast with open-loop
plan execution: a BT is *re-decided* continuously. That is what "reactive" means concretely.

### The node set (deliberately minimal)

| Node | Semantics | Where it's used |
|---|---|---|
| **Sequence** | "and then" — advance on SUCCESS, **fail fast**, `RUNNING` sticks to the current child | waypoint routes |
| **Fallback** | "else try" — succeed fast, try the next child on FAILURE | where recovery behaviors hang |
| **Action** | leaf wrapping `fn(ctx) → Status`; **all** world interaction lives here | goto, follow |

`Sequence` is a **memory** sequence: it stores `_current`, so a resumed tree doesn't re-run
finished children (the test proves a `setup` leaf runs once, not every tick). `ctx` is the
**blackboard** — a shared dict letting leaves pass data without globals.

### The two real trees

- **goto / patrol_leg / sweep_cell** → `Sequence` of goto leaves. Each leaf commands the
  backend every tick and returns `SUCCESS` inside `ARRIVE_TOL` (0.3 m).
- **follow_track** → a chase leaf that reads the **live track picture every tick**, commands a
  standoff point above the target, and returns:
  - `RUNNING` forever — *following has no natural end; it is preempted*, and
  - `FAILURE` when the track vanishes → which the mission node converts into a **replan
    trigger**.

That failure path is BT reactivity doing real work, not decoration.

## 3.3 Multi-agent allocation — CBBA (`allocator.py`, decision `021`)

### The options

| Option | Character | Verdict |
|---|---|---|
| Hungarian | optimal 1-1, `O(n³)` | textbook, exact; **centralized**, one task per drone, weak narrative |
| **CBBA** ✅ | decentralized bundle auction with consensus | the canonical swarm allocator (Choi/Brunet/How 2009); matches the "Distributed" thesis; most complex |
| Sequential auction | greedy market rounds | simple; no optimality story |
| MILP | exact optimization | slow, solver dependency, not real-time |
| Learned (GNN) | publishable | training burden; belongs in Step 4/5 if ever |

**Decision: CBBA** — straight to the deep piece. Hungarian remains trivially available later
as an *evaluation baseline* (optimality-gap numbers for the Step-4/5 ladder).

### The algorithm

Two phases alternate until a fixed point.

**Phase 1 — Bundle building (per drone, greedy).**

For every unclaimed-or-outbid task, and every **insertion position** in this drone's current
route, compute the **marginal** score:

```
marginal = pathScore(route with task inserted at pos) − pathScore(route)
```

Take the best one, but only if it **outbids the task's current global winner**. Insert it,
record the bid and the winner, repeat until the bundle is full or nothing is profitable.

A bid is a **marginal value** — what the task adds to *this* drone's route — so tasks that lie
"on the way" naturally earn high bids. That is the elegance.

**The score function:**

```
S = Σⱼ  λ^τⱼ · rewardⱼ        λ = 0.95 per second,  τⱼ = arrival time along the route
```

Rewards **discounted by arrival time**, with travel times from straight-line legs at cruise
speed. This is not arbitrary: time-discounted rewards are **diminishing marginal gain** —
adding a task never *increases* another task's marginal value — and **DMG is the property
CBBA's convergence and bounded-suboptimality guarantee requires.** Break DMG and the auction
can oscillate forever.

**Phase 2 — Consensus + bundle truncation.**

Drones exchange bids. The highest bid per task wins (ties → lower drone id). A drone that
**loses** a task drops it **and every task it added afterwards.**

That truncation rule is the subtle heart of CBBA: later additions were scored *assuming the
lost task was in the path*, so their bids are stale. Without truncation, the auction never
converges.

**Capability filtering** is a *hard constraint applied before scoring* (decision `007`): a
radar-only drone is never even considered for a camera task. Pinned by
`test_cbba_capability_constraint`.

**Documented simplification:** fully-connected synchronous communication, so consensus is a
global max per task per round (≤ `N_tasks + 1` rounds). The full Choi conflict-resolution
table — 36 rules with message timestamps — is only needed for **multi-hop / asynchronous**
networks, and slots in behind the same interface when comms get realistic.

## 3.4 Coverage — Voronoi partitioning (`coverage.py`, decision `022`)

### The options

| Option | Idea | Verdict |
|---|---|---|
| Boustrophedon | serpentine sweep, split into strips | simplest, complete; "uninteresting", no multi-agent structure |
| **Voronoi** ✅ | each point belongs to its nearest drone; each drone sweeps its cell | elegant, naturally decentralized, travel-balanced by construction |
| Frontier | target the known/unknown boundary | **the** approach for unknown maps; needs an occupancy map |
| Learned | — | Step 4/5 territory |

**Decision: Voronoi, implemented grid-based** — and the grid choice is the interesting part.

### Why grid-based

```
V_i = { q : ‖q − p_i‖ ≤ ‖q − p_j‖  ∀j }        the Voronoi cell of drone i
```

Rather than compute exact geometric cells (and clip them to an arbitrary polygon — fiddly, and
a library dependency), the area is **discretized into sample points**; each point joins its
nearest drone seed (`argmin` over distances, vectorized); each cell is serpentine-ordered into
a route. Point-in-polygon is 10 lines of ray casting (odd crossings = inside).

Three reasons:
1. Zero geometry dependencies.
2. Arbitrary polygons for free.
3. **Frontier exploration is also grid-based** (occupancy grids).

That third reason is a direct answer to the requirement *"I might want to move to unknown maps
with physical hardware later — can we switch?"* — **yes**: `FrontierCoverage` swaps in behind
the same `CoveragePlanner` protocol, reusing this exact discretization. Known map = partition
once; unknown map = re-plan toward frontiers as the map grows.

**Theory:** Voronoi coverage control (Cortés et al.) gives each agent the region it dominates
by distance — travel-balanced by construction and decentralizable (each drone could compute
its own cell from peer positions). We partition **once per intent**; continuous Lloyd-style
re-centering is the research-grade extension.

Note the layering: the 022 decision was the **partition** strategy. The serpentine route
*inside* each cell is a boustrophedon — an implementation detail, not a competing choice.

## 3.5 Execution and the dynamics seam (`executor.py`)

### `DroneBackend` — the fidelity ladder's seam

Three methods: `goto(drone_id, waypoint)`, `pose(drone_id)`, `step(dt)`.

`KinematicBackend` integrates constant-speed point-mass motion, mutating the **shared**
`DroneState` objects — so `WorldState` is the single source of truth and Isaac merely renders
`/drone_poses`.

**This inverts at the quadrotor upgrade** (decision `008`'s ladder): the sim will own the
state, and the backend becomes a command bridge. Nothing above the interface changes — that is
the entire point of writing it as a Protocol now.

**The ladder** (user's plan, recorded in `008`):
1. **Kinematic prims** (now, Steps 1–3) — fast iteration for the classical stack.
2. **RL tested on kinematics first** (Step 4 entry) — cheap rollouts, fast debugging.
3. **Switch to real quadrotor dynamics + assets**, then **re-test RL** and **generate all
   Step-5 learned layer training data on assets, never on kinematics** — the learned stack that matters
   must never be trained on toy dynamics.

### The Executor

Per-drone task queues in CBBA path order; one tree at a time per drone; `SUCCESS` pops the next
task, `FAILURE` logs and moves on. `assign()` **replaces** all queues — a replan preempts
everything, because trees are cheap and rebuilding beats patching.

## 3.6 The live brain (`mission_node.py`)

The ROS 2 node that runs the whole L2 loop at 10 Hz:

```
in:  /intent (StructuredIntent)      /tracks (L1)        /engagement_decisions (L4)
out: /drone_poses (→ Isaac, COP)     /mission_status     /engagement_proposals (→ L4)
```

**Replan triggers** (minimal and documented):
- a **new intent** arrives → full re-decompose + re-allocate
- a **`follow_track` task fails** → its track vanished → same

Completed task ids are excluded on replan, so finished work never re-runs.

**Engagement rule (demo):** a follower holding within `ENGAGE_RANGE` (3 m) emits **one**
`EngagementProposal` per track. The action itself is out of scope — the point is exercising the
L4 approval loop and the audit trail.

---

# Part 4 — L3 (COP) and L4 (Human-on-the-Loop)

## 4.1 The viewer decision (`030`/`031`)

| Option | Verdict |
|---|---|
| Three.js + React | known from APE_GCS, full control; every scene primitive hand-built |
| **Rerun** ✅ | purpose-built robotics/physical-AI visualization: 3-D, time-scrubbing, Python SDK |
| Foxglove | ROS-native panels; heavier, less bespoke |
| Unity + WebGL | prettiest; wrong effort allocation |

**Decision: Rerun** — tooling aligned with the physical-AI ecosystem, and weeks of scene
plumbing saved.

Choosing it **collapsed the transport fork too**: visualization rides the **Rerun SDK**, while
**ROS 2 + the `040` schemas remain the inter-layer transport.** Rerun is a *tap*, never a hop
in the pipeline.

**The honest limitation, recorded up front:** Rerun is a **viewer, not a control surface** —
it hosts no buttons or forms. So operator *input* needs its own channel: a CLI console today,
with Three.js documented as the fallback if the COP ever must be a bespoke interactive
terminal.

## 4.2 The COP bridge (`cop/rerun_bridge.py`)

Subscribes the system's own topics and logs entities:

- `/tracks` → `Boxes3D` sized by **fused extent**, labeled `class(confidence)`, plus velocity
  `Arrows3D`
- `/drone_poses` → labeled `Points3D`
- `/mission_status` → a text panel

The timeline is **sim time**, so Rerun's scrubber **replays the mission**.

*WSL note:* the native viewer can't `spawn()` there. The bridge serves gRPC + the **web
viewer**, and mirrored networking means the Windows browser reaches it at
`http://localhost:9090/?url=<grpc-uri>`.

## 4.3 The operator console (`cop/console.py`)

One-shot CLI → `StructuredIntent` → `/intent`:

```bash
python3 -m mini_lattice.cop.console patrol --area 0,0 20,0 20,20 0,20
python3 -m mini_lattice.cop.console track  --track-id 0
```

`parse_intent` is pure (testable); the ROS wrapper **blocks until a subscriber matches** before
publishing — a lesson learned live (§6).

## 4.4 The approval gate (`hol/gate.py`)

### Structure

**Pure logic separated from middleware.** `ApprovalGate` is a dependency-free state machine
(unit-tested); `main()` is the terminal console around it. *Safety logic must be testable
without ROS.*

### The state machine

```
                    ┌──────────┐  decide(approve=True)   ┌──────────┐
   submit() ──────▶ │ PENDING  │ ──────────────────────▶ │ APPROVED │
                    │          │ ──────────────────────▶ │ DENIED   │
                    └──────────┘  decide(approve=False)  └──────────┘
                          │
                          │  tick(now) with now − submitted ≥ deadline_s
                          ▼
                    ┌──────────────────────┐
                    │ AUTO-DENIED (timeout)│   ← the fail-safe
                    └──────────────────────┘
```

**The invariant that matters: silence never authorizes.** A human who says nothing has *not*
approved. That single property is the entire L4 narrative (and the DoD 3000.09 /
"meaningful human control" framing the design doc points at).

Double-deciding raises with the pending list — never silently pass.

### The audit trail

Every transition appends — in memory and to **append-only JSONL** — `{event, proposal_id,
action, target_track_id, rationale, operator, time}`. This is the "explain any engagement after
the fact" artifact.

**It proved itself live.** During the demo, the gate wasn't listening when the organic proposal
fired:

```json
{"event":"auto_denied_timeout","proposal_id":"p0","rationale":"d1 holding 2.9 m from track 0 (vehicle)",...}
{"event":"proposed","proposal_id":"p-resend-2",...}
{"event":"approved","proposal_id":"p-resend-2","operator":"console",...}
```

The fail-safe fired unprompted, then a real human approval was recorded with full provenance.
That JSONL fragment is the single best artifact in the project for a safety conversation.

## 4.5 The interfaces (decision `040`, `schemas.py`)

Locked **before** any layer code, per the design doc's rule: *"Interface stability >
implementation elegance."*

| Message | Direction | Notable content |
|---|---|---|
| `TrackMsg` | L1 → L2/L3 | position, velocity, extent, `position_cov`, class + **DS-native `class_beliefs`** |
| `StructuredIntent` | operator → L2 | verb, target (track/polygon/point), priority, deadline |
| `TaskAssignment` | L2 → drones | task, drone, waypoints, **`intent_id` provenance** |
| `EngagementProposal` | L2 → L4 | action, target, rationale, `deadline_s` |

Pydantic models are the source of truth; the wire codec is JSON (`model_dump_json`), interim
inside `std_msgs/String`, upgradeable to generated `.msg`/IDL without touching the definitions.
DS masses cross the wire via `encode_mass`/`decode_mass` (frozensets ↔ `"person|vehicle"`).

Two design points: the module is **top-level** (no layer owns it), and schema changes from here
are **versioned decisions** — this is the expensive-to-change surface, because generated
datasets will eventually depend on it.

---

# Part 5 — The Decision Ledger

*Every decision, what was chosen, and the one-line reason. Full records in `docs/decisions/`.*

| # | Decision | Chosen | Why (short) | Rejected |
|---|---|---|---|---|
| **000** | Scope & layer depth | **L1–L3 deep-ish, L4 light** | two interview-defensible layers + a visible demo | edge-only, autonomy-only, full-stack-thin |
| **001** | Sim environment | **Isaac Sim on Windows + ROS 2 across WSL** | best sensor realism + industry signal; sim-to-real = driver swap | pybullet, Gazebo, custom, Isaac-in-WSL |
| **007** | Sensor modalities | **radar + EO/IR camera + lidar**, phased; fixed **and** drone-mounted; heterogeneous loadouts | each covers the others' failure modes; richest fusion story | single-modality, camera+lidar only |
| **008** | Vision & staging | **6 steps → staged build to a learned autonomy layer**; each ships alone | anti-monster discipline; the baseline ladder | monolithic build |
| **010** | State estimator | **UKF** (IMM as documented upgrade) | 3 heterogeneous `h(x)` → no Jacobians to maintain | EKF, IMM now, particle filter |
| **011** | Data association | **JPDA** (MHT fallback) | soft association survives clutter; upgrade is an interface swap | GNN/Hungarian, MHT now, learned |
| **012** | Classification fusion | **Dempster-Shafer** | sensors have different vocabularies; ignorance ≠ uniform prior | naive Bayes, confusion matrices, voting |
| **013** | Observation & sensing | **radar-primary depth, parallax fallback; 9-D full-extent state; yaw-from-velocity** (+ later: **3-D radar**) | breaks scale–depth ambiguity; cheap (a)→(c) migration | point target, scalar size, W+H, yaw-in-state now |
| **014** | Covariance strategy | **SR-UKF** default | structural PSD safety + best stress numbers; cost is affordable | shortcut+guards, Joseph |
| **015** | DS conflict + decision rule | **conflict-weighted discounting** + **pignistic** | reliability semantics fit heterogeneous sensors; BetP feeds L3/L4 | Yager, Murphy; max-Pl, max-Bel |
| **020** | Task decomposer | **HTN plans → BT executes**, LLM seam reserved | planning story + execution story; the LLM slot is a certainty | BT-only, LLM-now, PDDL |
| **021** | Allocator | **CBBA** | canonical decentralized swarm allocation; the "deep piece" | Hungarian, auction, MILP, GNN |
| **022** | Coverage planner | **Voronoi, grid-based** | elegant + decentralizable; grid = frontier-ready for unknown maps | boustrophedon, frontier now, learned |
| **030/031** | COP viewer + transport | **Rerun** (+ ROS 2 inter-layer, CLI for input) | built for physical AI; collapses the transport fork | Three.js, Foxglove, Unity |
| **040** | Interface schemas | **4 pydantic messages over JSON/ROS 2** | interfaces stable, implementations swappable | ad-hoc dicts, .msg-first |

**Backlog decisions** (`docs/backlog.md`, prefixed `D-B`): D-B1 id-keyed sensor union; D-B2 →
became `014`; D-B3/B4 → became `015`; D-B5 radar elevation (added); D-B7 DS-native
`class_beliefs`; D-B11 `Q_ext` tuning. Still open: D-B6 variable-dt prediction, D-B8 hardware
purchase, D-B9 roadmap slots, D-B10 the direction of the learned layer.

---

# Part 6 — Every Bug and What It Taught

*The most valuable section for interviews. Each entry: symptom → cause → lesson.*

### 6.1 The shared-IP DDS trap
**Symptom:** everything runs, discovery succeeds, zero data crosses.
**Cause:** mirrored networking gives both OSes the same IP → Fast DDS picks shared-memory
transport and advertises own-IP locators.
**Lesson:** *discovery ≠ data.* When topics are visible but silent, suspect **transport
selection**, not discovery. Diagnose by instrumenting *below* the failing layer — a raw UDP
socket proved the road existed.

### 6.2 The sigma-point rows/columns bug
**Symptom:** occasional non-PSD covariance; filter "worked" otherwise.
**Cause:** sigma offsets taken from **rows** of numpy's lower Cholesky factor → reconstructs
`LᵀL`, not `LLᵀ = P`. Same eigenvalues, trace, determinant — **rotated correlation directions.**
**Lesson:** a "numerical stability" symptom can be a **correctness bug in disguise**. Test
*reconstruction*, not health. Know your library's factor convention (numpy lower, scipy upper).

### 6.3 Sigma weights at n=9
**Symptom:** `LinAlgError: Matrix is not positive definite` on the first real camera update.
**Cause:** `α=1e-3, κ=0` → `λ ≈ −9` → center weights ±10⁶.
**Lesson:** **sigma-point tuning is dimension-dependent.** Textbook defaults tested at n=3 are
hostile at n=9. `α=1, κ=1` makes all weights positive → PSD by construction.

### 6.4 Zero-velocity initialization
**Symptom:** the camera update overshot into divergence on newborn tracks.
**Cause:** option-A yaw comes from velocity; `v=0` with a wide prior makes sigma points span
*every heading* → wildly varying projected widths → a meaningless `Pxz`/`S` → overshooting `K`.
**Lesson:** **state couplings reach into initialization design.** Two-point differencing (the
standard track-initiation recipe) exists for exactly this reason.

### 6.5 Transient vs persistent repair
**Symptom:** the live run looked fine; the new test suite immediately found `P` with
eigenvalue −0.03.
**Cause:** the eigenvalue-floor repair lived only inside sigma-point *generation* — the
**stored** `P` stayed sick.
**Lesson:** **repair at the source of the invariant violation, not the point of consumption.**
And: *one successful live run is one seed, not a proof* — the test found the counterexample in
seconds.

### 6.6 The smug filter (`Q_ext`)
**Symptom:** NEES ≈ 9,800 vs an ideal 9.
**Cause:** `Q_ext = 1e-8` collapsed extent covariance while the estimate stayed biased.
**Lesson:** **`Q` is not only "how much the world moves" — it is how much you admit your model
is wrong.** Tune it against NEES. Also: **split NEES per block**; the aggregate hid that
position was consistent all along.

### 6.7 `python.bat` has no ROS environment
**Symptom:** `rcutils.dll` not found; bridge dead in standalone scripts, fine in the GUI.
**Cause:** `isaac-sim.bat` calls `setup_ros_env.bat`; **`python.bat` does not.**
**Lesson:** when the GUI works and your script doesn't, **diff the launchers.** Read the
vendor's own scripts.

### 6.8 OmniGraph disables erroring nodes
**Symptom:** `/tf` published for seconds, then went silent forever. No errors.
**Cause:** OmniGraph disables a node that errors during execution; the TF branch died while
the independent clock branch kept ticking.
**Lesson:** **silent partial failure** is the worst failure mode. Rule adopted: *in-scene rclpy
for data the script computes; OmniGraph publishers only for data OmniGraph computes.*

### 6.9 Port-less initial peers cap discovery at 5 participants
**Symptom:** with six nodes up, a newly launched node was undiscoverable — publisher saw 0
subscribers forever.
**Cause:** a port-less initial-peers locator probes only participant ids **0–4**.
**Lesson:** in unicast-peer setups, **size the peer port list to the node population**
(`7410 + 2·id`).

### 6.10 `rclpy.Node.executor` is a property
**Symptom:** `AttributeError: 'Executor' object has no attribute 'add_node'` at construction.
**Cause:** assigning `self.executor` on a `Node` subclass hits rclpy's setter.
**Lesson:** know your base class's namespace before naming attributes.

### 6.11 One-shot publishers must wait for discovery
**Symptom:** the console reported "sent"; L2 never received anything.
**Cause:** with unicast initial peers, matching takes seconds; publishing before a subscriber
matches drops the message **silently**.
**Lesson:** block on `get_subscription_count() > 0` (with a timeout and a real error message)
in short-lived publishers.

### 6.12 `ROS_SUPER_CLIENT` corrupts the Humble daemon
**Symptom:** every `ros2` CLI call → `!rclpy.ok()`, surviving daemon restarts *and* a WSL reboot.
**Cause:** the env var kept respawning a broken daemon.
**Lesson:** `--no-daemon` isolates daemon bugs from DDS bugs in ~10 seconds. Prefer raw
`rclpy` probe scripts as ground truth over the CLI.

### 6.13 Also worth remembering
- **Kit stdout is block-buffered** — logs "freeze" at `app ready` while the app runs fine.
  Check CPU delta, not the log, for liveness.
- **Isaac: running ≠ publishing.** Needs a publisher *and* a playing timeline.
- **`ros2 topic echo` can be silent where rclpy receives** — the CLI adds daemon/QoS layers
  that fail independently.
- **The shipped `standalone_examples/` are the API ground truth** for Isaac 6.0.1 — newer than
  any model's training data. Every node type in the scene was lifted from them, not guessed.

---

# Part 7 — Interview Prep: What You Must Derive From Memory

The rule: *you must be able to reconstruct any core component on a whiteboard.* Here's the
checklist, by topic.

### Estimation
- [ ] Derive the Kalman update (`K = PHᵀ(HPHᵀ+R)⁻¹`) and explain it as precision-weighted fusion.
- [ ] Explain the **unscented transform**: why approximating a distribution beats approximating
      a function; 2nd-order accuracy vs the EKF's 1st.
- [ ] Write the sigma-point equations *including* the weights, and explain **α, β, κ**.
- [ ] Explain why offsets are the **columns** of the Cholesky factor (and tell the bug story).
- [ ] Explain why `P − KSKᵀ` can go indefinite, what **Joseph form** buys, and how the
      **SR-UKF** removes the failure structurally.
- [ ] **UKF vs EKF vs IMM vs particle filter** — when each wins.
- [ ] **NEES/NIS**: definitions, expected values, what over/under-confidence means, how you'd
      tune `Q` with them.

### Observability
- [ ] Explain the **scale–depth ambiguity** and three independent ways to break it.
- [ ] Explain why a bearings-only sensor can't localize without motion (and what "the observer
      must out-maneuver the target" means).
- [ ] Explain what an **unobservable subspace** is and why adding a state DOF you can't observe
      is *worse than useless*.

### Association & tracking
- [ ] Derive the **Mahalanobis gate** and explain why the χ² DOF is the *measurement* dimension.
- [ ] Derive **PDA β weights** from Bayes over association events, including the null hypothesis.
- [ ] **GNN vs JPDA vs MHT** — complexity and failure modes (clutter snap; identity merge on
      crossings).
- [ ] Explain the **track lifecycle** state machine and the continuity/false-track trade.
- [ ] Name what our PDA *omits* (joint events; covariance inflation) — knowing your own
      simplifications is the strongest possible answer.

### Classification fusion
- [ ] Derive **Dempster's rule**; define `Bel`, `Pl`, and the interval.
- [ ] Walk **Zadeh's paradox** numerically, then explain **discounting vs Yager vs Murphy** in
      terms of *semantics* (unreliability vs ignorance vs averaging).
- [ ] Define **BetP** and prove `Bel ≤ BetP ≤ Pl`.
- [ ] Answer "why not Bayesian?" in one sentence: *ignorance ≠ uniform prior.*

### Planning & multi-agent
- [ ] Explain **HTN** decomposition (methods, preconditions, state-dependent branching).
- [ ] Explain **BT** semantics: tick, `RUNNING`, memory vs memoryless, sequence vs fallback,
      and why reactivity beats scripted execution.
- [ ] Explain **CBBA** end-to-end: bundle building, marginal bids, consensus, **truncation**,
      and *why DMG scores are required* for convergence.
- [ ] Compare CBBA to Hungarian/auction/MILP on optimality, decentralization, and complexity.
- [ ] Explain **Voronoi coverage control** and why grid discretization unifies it with frontier
      exploration.

### Systems
- [ ] Sketch the L1→L2→L3→L4 architecture from memory.
- [ ] Explain **DDS discovery vs transport** and the shared-IP trap.
- [ ] Explain how you'd **scale to 100 drones and 10 sensors**: where the bottlenecks are
      (centralized fusion, CBBA consensus rounds, COP bandwidth), and what goes decentralized
      first.
- [ ] Justify **every algorithm choice against its alternatives** — that's what the decision
      docs are for. Internalize them.

---

# Part 8 — Where This Goes Next

### Immediate (unblocked, no decisions needed)
- **Multi-target scene** — 2–3 movers + clutter. This is the trigger for **joint JPDA**
  (overlapping gates) and makes the association work earn its keep.
- **MOTA / MOTP / IDF1 harness** against Isaac ground truth — also the metric substrate for
  comparing later approaches on equal footing.
- **Publish `TrackMsg` properly** instead of interim JSON (finish `040` adoption).
- **Cleanup:** the original `tests/edge/test_ukf.py` vs the current `tests/test_ukf.py`;
  `scripts/discovery-server.sh` is now vestigial.

### Step 3 — Language front-end
Speech → transcription → a model turning natural language into the same `StructuredIntent` the
system already understands. **The seam already exists**: `LLMDecomposer` implements the same
`Decomposer` protocol. This becomes the **symbolic baseline** later approaches are measured
against — so it is not a step to skip.

### Step 4 — Learned control policy
A learned brain behind the same L2 interface, trained on kinematics first and re-tested on
real quadrotor dynamics after the fidelity-ladder switch. Expect the usual multi-agent
learning pain: non-stationarity, credit assignment, reward design.

### Step 5 — Learned autonomy layer
End-to-end learned mission control running inside the same architecture, trained on
simulator-generated demonstrations. That dataset is the project's key enabling asset, and it
needs cloud GPUs — the 8 GB laptop won't train anything at that scale.

### The through-line
Every step lands behind the **same interface**, on the **same tasks**, measured by the **same
metrics**. That constraint is what makes the classical, learned-from-scratch, and pretrained
approaches genuinely comparable — and it is why the schemas were frozen before any layer code
was written.

# Appendix A — File Map

```
src/mini_lattice/
├── schemas.py                 040: the 4 cross-layer messages + DS mass codec
├── edge/                      ── L1: PERCEPTION ──
│   ├── types.py               Detection, TrackState, Track
│   ├── config.py              013/D-B1: 9-D state, id-keyed sensor union, per-sensor χ² gates
│   ├── observation.py         013: h_camera (8-corner pinhole), h_radar (r/az/el/doppler)
│   ├── ukf.py                 010: sigma points, predict, update (shortcut | joseph)
│   ├── srukf.py               014: square-root UKF (QR + rank-1 update/downdate)
│   ├── filters.py             014: make_filter / initial_state — the estimator seam
│   ├── jpda.py                011: gating, PDA β weights, combined innovation
│   ├── classification.py      012/015: DS masses, discounting, Dempster, Bel/Pl, BetP
│   ├── tracker.py             the L1 loop: predict → associate → update → classify → lifecycle
│   └── thin_slice_node.py     live harness: Isaac truth → synthesized detections → tracker
├── autonomy/                  ── L2: AUTONOMY ──
│   ├── world_state.py         fleet + track bookkeeping
│   ├── decomposer.py          020: Decomposer protocol (THE LLM seam), HTN methods
│   ├── coverage.py            022: CoveragePlanner protocol (frontier seam), Voronoi
│   ├── allocator.py           021: CBBA
│   ├── bt.py                  020: Sequence / Fallback / Action, RUNNING memory
│   ├── executor.py            008: DroneBackend seam (THE dynamics seam) + Executor
│   └── mission_node.py        the live L2 brain at 10 Hz
├── cop/                       ── L3 ──
│   ├── console.py             operator intents (CLI)
│   └── rerun_bridge.py        030: the 3-D picture
└── hol/gate.py                ── L4: approval state machine + JSONL audit ──

sim/scenes/thin_slice.py       the Isaac scene (target + fleet prims)
config/fastdds-loopback.xml    THE bridge fix — UDP-only, loopback, explicit peer ports
benchmarks/                    014 bake-off, D-B11 NEES sweep
docs/                          decisions/, backlog.md, bridge log, sigma-bug, this file
explanations/                  line-by-line study docs (gitignored)
```

# Appendix B — Run Book

```bash
# ── WSL: every shell that talks to Isaac ──
source scripts/ros-env.sh
bash scripts/verify-bridge.sh                      # checks discovery AND data

# ── Windows: the sim ──
C:\IsaacSim\run_scene.bat C:\IsaacSim\thin_slice.py

# ── The full demo (each in its own terminal) ──
python3 src/mini_lattice/edge/thin_slice_node.py       # L1
python3 src/mini_lattice/autonomy/mission_node.py      # L2
python3 src/mini_lattice/cop/rerun_bridge.py           # L3 → http://localhost:9090/?url=...
python3 src/mini_lattice/hol/gate.py                   # L4 (interactive y/n)
python3 -m mini_lattice.cop.console patrol --area 0,0 20,0 20,20 0,20
python3 -m mini_lattice.cop.console track --track-id 0

# ── Tests & benchmarks ──
PYTHONPATH=src .venv/bin/python -m pytest tests/ -q
PYTHONPATH=src .venv/bin/python benchmarks/covariance_strategies.py
PYTHONPATH=src .venv/bin/python benchmarks/nees_diagnosis.py
```

*(All node commands need `PYTHONPATH="src:$PYTHONPATH"` — **prepend**, never overwrite, or you
lose `rclpy`.)*

# Appendix C — The Numbers

| Quantity | Value | Source |
|---|---|---|
| State dimension | 9 | decision `013` |
| Sigma points | 19 (`2n+1`) | `α=1, κ=1 → λ=1`, all weights positive |
| Camera measurement | `[u,v,w,h]`, 4-DOF, gate χ²₉₅ = 9.49 | `config.py` |
| Radar measurement | `[r,az,el,doppler]`, 4-DOF, gate 9.49 | `013` amendment |
| Process noise | pos 1e-3, vel 1e-2, **extent 1e-3** | D-B11 NEES sweep |
| Live tracking error | 0.1–0.3 m at ~400 m range | 683-cycle run |
| Extent NEES | 9,770 → **25** after `Q_ext` fix | `nees_diagnosis.py` |
| SR-UKF cost | ~2.2 ms/step vs 1.7 ms shortcut | `covariance_strategies.py` |
| CBBA discount | λ = 0.95 / s, bundle ≤ 5 | `allocator.py` |
| Track lifecycle | confirm 3, delete after 5 misses | `config.py` |
| L4 fail-safe | auto-deny at `deadline_s` | `gate.py` |
| Tests | 34 passing | `tests/` |
| Source | ~2,750 lines src+sim, ~700 tests, ~1,500 docs | — |

# Appendix D — Glossary

**Association** — deciding which measurement belongs to which track.
**BetP** — pignistic probability; DS masses flattened for decision-making.
**CBBA** — Consensus-Based Bundle Algorithm; decentralized multi-task auction.
**DDS** — the pub/sub middleware under ROS 2.
**DMG** — diminishing marginal gain; the property CBBA's convergence needs.
**Focal element** — a subset of classes carrying DS mass.
**Gating** — rejecting measurements too far (in Mahalanobis distance) from a prediction.
**HTN** — Hierarchical Task Network; recursive decomposition planning.
**Innovation** — `z − z_pred`, the measurement surprise.
**Mahalanobis distance** — covariance-normalized distance, `√(dᵀS⁻¹d)`.
**NEES / NIS** — normalized error / innovation squared; filter consistency metrics.
**Observability** — whether measurements can determine a state; unobservable states drift.
**PSD** — positive semi-definite; a valid covariance must be.
**Sigma points** — the deterministic sample set of the unscented transform.
**Θ (Theta)** — the frame of discernment; mass on Θ means total ignorance.
**Unscented transform** — propagating a distribution through a nonlinearity via sigma points.
**Voronoi cell** — the region closer to one seed than to any other.

---

*End of deep dive. Companion documents: `docs/decisions/` (the 16 records),
`docs/isaac-wsl-ros2-bridge.md` (infrastructure log), `docs/ukf-sigma-point-bug.md` (bug
anatomy), `docs/backlog.md` (open items), `explanations/` (line-by-line).*
