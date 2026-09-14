# 019 — Ego-pose estimation: state, camera mount, filter architecture, odometry source

**Status:** Accepted
**Date:** 2026-09-13
**Depends on:** `017` (camera on the drones), `018` (SLAMMOT with prior-mapped signs)

## Context

`018` left the estimator's shape open. Four choices had to be made before any of it could be
built, and each changes the code that follows: what a drone's own pose *is*, where its camera
points, whether ego pose and targets share one filter, and where drift comes from in a simulation
whose drones are kinematic points with exactly known positions.

## Options considered

**A. What the ego state is**

| Option | Pro | Con |
|---|---|---|
| Position only; camera heading from the direction of travel | Smallest change; mirrors how targets get yaw in `013` | Heading undefined at hover, which drones do constantly; heading error never estimated |
| Position + yaw, roll and pitch assumed level | The four quantities an IMU cannot observe on its own — gravity makes roll and pitch observable, position and heading about the vertical are not | Yaw wraps at ±π and cannot be averaged naively; the wrapping problem is already an open backlog item |
| **Full 6-DoF pose** ← chosen | Complete; no assumption about the platform staying level | A quaternion cannot be averaged naively either — the filter has to work on an error state |

Heading matters at range: a 1° heading error displaces a landmark 100 m away by 1.75 m. A filter
that never estimates heading treats distant landmarks as more trustworthy than they are.

**B. Camera mount**

| Option | Pro | Con |
|---|---|---|
| Straight down | Simple ground footprint; parked vehicles seen from above show size and colour clearly | Road signs are vertical plates — seen nearly edge-on from above, so the prior-mapped anchor would mostly go unobserved |
| **Forward, pitched 30° down** ← chosen | Sees sign faces at a distance, where their standard size gives range; also sees vehicles | Trapezoidal ground footprint with uneven resolution; camera heading is the drone's heading |
| Stabilised gimbal | Camera direction independent of heading | Pointing becomes its own control problem (the RL option not chosen in `023`) |

**C. Filter architecture**

| Option | Pro | Con |
|---|---|---|
| One joint filter (ego, landmarks, targets) | Ego–target error correlation represented exactly | State grows with every drone, landmark and target; UKF cost grows with the cube of state size; a moving vehicle wrongly held as a landmark drags the ego poses with it |
| Decoupled: SLAM for ego + landmarks, existing tracker consumes the SLAM pose (Wang, Thorpe et al. 2007) | Tracker barely changes; state size bounded | Tracker reuses one pose estimate across many updates as if its errors were independent — counts the same information repeatedly, so target covariance ends up overconfident |
| **Decoupled, with a Schmidt–Kalman ("consider") target filter** ← chosen | Target covariance carries ego-pose uncertainty without updating it, and keeps the cross-covariance so repeated observations from the same biased pose are not counted as independent | More complex than the plain split; targets no longer improve the ego pose (matters less with prior-mapped signs anchoring the map) |

**D. Odometry source**

| Option | Pro | Con |
|---|---|---|
| Integrate commanded velocity plus noise | Simplest; drift grows with distance, like odometry | Synthetic, and the noise level is chosen |
| **Simulated IMU** (accelerometer and gyroscope with drifting biases, integrated) ← chosen | Realistic drift structure: bias makes position error grow with time squared; the standard input for a 6-DoF estimator | The kinematic backend turns instantly at waypoints — infinite acceleration — so trajectories need an acceleration-limited backend first |
| No odometry: constant-velocity prediction corrected by landmarks only | No noise model to justify | Uncertainty explodes between sightings; with few landmarks the filter diverges |

## Decision

**Full 6-DoF ego pose, camera fixed forward and pitched 30° down, a decoupled architecture with a
Schmidt–Kalman consider update in the target tracker, and a simulated IMU as the odometry source.**

## Choices derived from the decision

Recorded here because they follow from the four choices above rather than being open, and flagged
so they can be reopened if the reasoning does not hold:

- **Error-state formulation, sigma-point flavour.** A quaternion in the state forces an error-state
  filter. `010` chose the UKF for target tracking so that each new `h(x)` needs no Jacobian; the
  ego filter's measurement models (a box landmark projected through a camera at the ego pose) are
  exactly that shape, so the same reasoning gives an **error-state UKF** — sigma points on the
  15-D error state (position, velocity, rotation vector, accelerometer bias, gyroscope bias),
  applied to the nominal state by a retraction (Solà 2017 for the error-state conventions; the
  unscented-on-manifold treatment follows the same structure). The alternative is the industry
  error-state EKF, which needs the IMU and camera Jacobians derived by hand.
- **One ego filter per drone, run in the L1 process.** Each drone estimates its own pose; the
  prior sign map is shared by construction. Vehicle landmarks are **not** shared between drones —
  a documented simplification, not a design position.
- **Vehicle landmarks initialise by delayed two-view triangulation.** A monocular box gives a
  bearing but not a range (vehicle size is not standard, unlike a sign). A candidate becomes a
  landmark only once seen from two viewpoints with enough parallax. Inverse-depth
  parameterisation is the usable-immediately alternative; a class-size prior is the practical
  shortcut. Neither was chosen.
- **The prior map is exact**, with a configurable inflation of sign-measurement noise standing
  in for map error. Surveying belongs to another project.
- **Covariance form, not square-root, for the ego filter.** `014` chose the square-root form
  for the target tracker, whose state never changes size. The ego state grows and shrinks as
  vehicle landmarks are added and dropped, which is a block append or a row/column deletion on a
  covariance and a re-factorisation on a factor. Covariance form with the same eigenvalue-floored
  repair (and the same `repairs` counter) is the pragmatic choice; the tests assert zero repairs.
- **An acceleration- and yaw-rate-limited backend** (`SmoothBackend`) replaces the snap-to-speed
  kinematic backend wherever an IMU is synthesised, and keeps the platform level. It implements
  the same `DroneBackend` Protocol, so nothing above the seam changes.

## Consequences

- New: quaternion utilities, a shared scene description (signs, vehicles, fleet, target), the
  smooth backend, the IMU synthesiser, the ego filter, the consider update, the SLAMMOT front end.
- `DroneState` gains an orientation and an estimated pose with covariance — a cross-layer schema
  change recorded in `041`.
- Coupled-error accounting stays approximate in one place: the consider block treats ego-pose
  error as constant between tracker cycles rather than propagating its own dynamics. Standard for
  slowly varying consider parameters; stated here so it is not mistaken for exact.
  *Implementation note (2026-09-13):* the cross-covariance is carried through every update on
  the target side (radar, other cameras) and through the ego filter's own corrections on the
  pose side, using the covariance ratio `P_after P_before⁻¹` as the implicit `(I − K H)`. Both
  were needed to keep the joint covariance positive definite; neither is a change to the
  decision, only to how it is honoured.
- Landmark maps are per drone. Sharing them is the first thing to add if drones ever need to agree
  on where a parked vehicle is.
- The live IMU rate equals whatever rate `/drone_poses` arrives at; the simulator-free harness
  runs the IMU at 100 Hz. The filter is rate-agnostic; the synthesiser is not, so the two differ
  only in how coarse the finite-difference acceleration is.
