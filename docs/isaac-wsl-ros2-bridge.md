# Isaac Sim (Windows) ↔ WSL2 ROS 2 bridge — architecture & debugging log

**Status:** WORKING, verified end-to-end 2026-07-29.
**Setup:** Windows 11 (build 26200) · WSL2 2.7.10 (Ubuntu 22.04, mirrored networking) ·
Isaac Sim 6.0.1 standalone at `C:\IsaacSim` · ROS 2 Humble on WSL · Fast DDS both sides.
**Context:** decision `001` (Isaac on Windows + ROS 2 across the WSL boundary, "W1"), after the
Ubuntu 20.04→22.04 migration (Foxy→Humble; decision 001's Foxy references are obsolete).

This document is the complete record: the final working configuration, then every issue hit on
the way, in order, with symptom → diagnosis → fix → theory. Nothing omitted.

---

## 1. The final working architecture

```
┌────────────── Windows 11 ──────────────┐   ┌──────────── WSL2 (Ubuntu 22.04) ───────────┐
│ Isaac Sim 6.0.1 (C:\IsaacSim)          │   │ ROS 2 Humble (/opt/ros/humble)             │
│  └─ run_scene.bat                      │   │  └─ scripts/ros-env.sh                     │
│      calls setup_ros_env.bat           │   │      sources setup.zsh/bash                │
│      (internal ROS 2 = jazzy,          │   │      RMW = rmw_fastrtps_cpp                │
│       Fast DDS 3.x)                    │   │      (Fast DDS 2.6)                        │
│      FASTRTPS/FASTDDS_DEFAULT_         │   │      FASTRTPS_DEFAULT_PROFILES_FILE=       │
│      PROFILES_FILE=fastdds-loopback.xml│   │      config/fastdds-loopback.xml           │
│                                        │   │                                            │
│  UDP unicast, 127.0.0.1 only, no SHM ──┼───┼── UDP unicast, 127.0.0.1 only, no SHM      │
└────────────────────────────────────────┘   └────────────────────────────────────────────┘
            both sides: identical XML; peer discovery via initialPeersList 127.0.0.1
            no multicast · no discovery server · no ROS_DISCOVERY_SERVER anywhere
```

**The one-paragraph summary:** WSL mirrored networking gives Windows and WSL the *same IP
address*. Default Fast DDS therefore (a) classifies the other side as *same-host* and selects
shared-memory transport, which cannot cross an OS boundary, and (b) advertises data locators
carrying that shared IP, which route back into the sender's own OS. Raw UDP over `127.0.0.1`
*does* cross the boundary in both directions (verified with plain sockets), so the fix is a
Fast DDS profile — identical on both sides — that forces UDP-only transport (kills SHM),
whitelists only the loopback interface (so all advertised locators are `127.0.0.1`), and uses
`initialPeersList = 127.0.0.1` for discovery (plain RTPS unicast peer discovery: no multicast,
no discovery server, and therefore also immune to the Fast DDS 2.6↔3.x discovery-server
protocol mismatch).

### Key files

| File | Side | Role |
|---|---|---|
| `config/fastdds-loopback.xml` | both (copy at `C:\IsaacSim\fastdds-loopback.xml`) | the transport/discovery profile that makes crossing work |
| `scripts/ros-env.sh` | WSL | env for any shell talking to Isaac (distro, RMW, profile) |
| `sim/run_scene.bat` (copy at `C:\IsaacSim\run_scene.bat`) | Windows | scene launcher: ROS env + profile + `python.bat` |
| `sim/scenes/thin_slice.py` (copy at `C:\IsaacSim\thin_slice.py`) | Windows | minimal custom scene: ground + CV-moving cube → `/clock`, `/tf` |

### To run

1. Windows: `C:\IsaacSim\run_scene.bat C:\IsaacSim\thin_slice.py` (add `> log 2>&1` to capture).
2. WSL: `source scripts/ros-env.sh`, then subscribe (rclpy, or `ros2 topic echo <topic> <type>`).

---

## 2. Chronological issue log

### Issue 1 — Decision: mirrored networking (replacing the original NAT plan)

Decision 001 predates the migration and assumed NAT + Fast DDS XML tricks. Windows build 26200
and WSL 2.7.10 support **mirrored networking** (`networkingMode=mirrored` in `.wslconfig`,
plus `[experimental] hostAddressLoopback=true`), which shares the Windows network stack with
WSL. Chosen for simplicity; WSL memory kept at 4 GB (Isaac needs Windows RAM headroom).
**Mirrored-mode signature** (how to confirm it took): WSL `eth0` carries the *same IPv4 as
Windows* (here `10.60.166.51`), extra mirrored adapters appear, and `10.255.255.254/32` sits
on `lo`. Verify with `ip -4 addr` vs `ipconfig`.
**Foreshadowing:** mirrored networking is the *root cause* of Issue 9 — sharing the IP breaks
DDS's assumptions about what "same host" means.

### Issue 2 — WSL interop appeared broken, then self-healed

**Symptom:** running any Windows exe from WSL (`cmd.exe`, `wsl.exe`) → `Exec format error`;
`/proc/sys/fs/binfmt_misc/WSLInterop` absent.
**Resolution:** after the `wsl --shutdown` for mirrored networking, the binfmt handler
registered and interop worked. No config change was made — the pre-restart state was stale.
**Why it mattered:** interop became the debugging workhorse — from WSL we could run
`tasklist.exe`, `NETSTAT.EXE`, `taskkill.exe`, `powershell.exe`, read `C:\IsaacSim` logs, and
launch/kill Isaac. The entire debugging session was driven from the WSL side this way.

### Issue 3 — First crossing attempt: the multicast wall

**Symptom:** Isaac GUI running the ROS 2 *perception* sample (stereo cams, IMUs, `/cmd_vel`,
`/clock`); WSL `ros2 topic list` showed only `/parameter_events` and `/rosout`.
**Diagnosis:** WSL's Fast DDS bound UDP `7400` (the DDS multicast discovery port). Windows
`netstat` showed Isaac holding only *unicast* ports (`7412/7413`) and **nothing on 7400**. The
two sides were announcing on channels the other never listened to.
**Theory:** DDS default discovery is **multicast** on `239.255.0.1:7400`. WSL mirrored
networking does *not* carry multicast across the Windows↔WSL boundary. So both stacks are
healthy, but discovery packets never meet. This is the fundamental reason any cross-boundary
DDS setup here needs a unicast discovery mechanism (server or initial peers).

### Issue 4 — Hyper-V firewall silently blocking inbound to WSL

**Symptom:** even planning unicast, inbound Windows→WSL traffic would die silently.
**Diagnosis:** `Get-NetFirewallHyperVVMSetting` showed the WSL VM's firewall policy
`DefaultInboundAction = Block` (VMCreatorId `{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}`). In
mirrored mode, traffic into WSL passes this **Hyper-V firewall**, separate from the classic
Windows Firewall profiles (which were also all enabled).
**Fix (admin PowerShell):**
```powershell
New-NetFirewallHyperVRule -Name "ROS2-DDS" -DisplayName "ROS2 DDS (WSL mirrored)" `
  -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' `
  -Protocol UDP -LocalPorts 7400-7700 -Action Allow
```
Scoped to the DDS port range rather than opening everything. Necessary but — as later issues
showed — not sufficient.

### Issue 5 — Fast DDS Discovery Server era (worked once, then betrayed us)

**Approach:** run `fastdds discovery -i 0 -l 0.0.0.0 -p 7500` on WSL; set
`ROS_DISCOVERY_SERVER=127.0.0.1:7500` on both sides (Windows via `setx` + full Isaac relaunch
so the env is inherited — env vars only reach processes started after they're set).
**Result:** the GUI perception sample's topics appeared in WSL *and data flowed* (`/clock` at
~16 Hz, `camera_info` with real intrinsics 960×600, fx≈478.9). Celebrated as the fix.
**Retrospective:** this success was partly luck. Discovery went through the server (unicast,
crossing fine), and the *data* likely crossed because mirrored mode routed own-IP unicast
packets to whichever OS held the destination port — fragile behavior that later stopped
working for the custom scene (see Issue 9). The discovery server was ultimately **retired**;
it also carried a hidden protocol risk: WSL Humble = Fast DDS **2.6**, Isaac jazzy = Fast DDS
**3.x**, and the discovery-server wire protocol is not guaranteed compatible across that major
version boundary (plain RTPS peer discovery *is*).

### Issue 6 — `ROS_SUPER_CLIENT=true` corrupts the Humble ros2 daemon

**Symptom:** every daemon-backed CLI call (`ros2 topic list`, `echo` without explicit type)
crashed with `xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">`. Persisted
across daemon restarts *and a full WSL restart* (the env var kept respawning a broken daemon).
**Diagnosis:** `ROS_SUPER_CLIENT=true` had been set so the CLI could see all server-registered
topics. Humble's ros2 daemon crashes internally in this mode; once one daemon instance is
poisoned, every CLI call to it faults. `ros2 topic list --no-daemon` worked fine, which
isolated the daemon (not ros2, not DDS) as the broken layer.
**Fix:** never set `ROS_SUPER_CLIENT`. When bypassing the daemon: `ros2 topic list
--no-daemon`; `ros2 topic echo <topic> <TYPE>` (explicit type skips daemon type lookup); or
raw `rclpy` probe scripts (most reliable — see Issue 10).

### Issue 7 — Standalone scripts: ROS 2 bridge fails to load (`rcutils.dll`)

**Symptom:** launching a scene via `C:\IsaacSim\python.bat scene.py` →
`Could not load the dynamic library from rcutils.dll` → `ROS2 Bridge startup failed` →
`OmniGraphError: unrecognized type 'isaacsim.ros2.bridge.ROS2PublishClock'`.
**Diagnosis:** reading the launchers: **`isaac-sim.bat` (GUI) calls `setup_ros_env.bat`;
`python.bat` does not.** So GUI sessions get a ROS 2 environment; standalone scripts get none,
and the bridge extension can't find its ROS libraries.
**Dead ends tried first (documented so they aren't retried):** forcing `ROS_DISTRO=humble` and
manually appending the internal Humble lib dir to `PATH` — with forward slashes, then
backslashes. Both still failed (`rcutils.dll` unresolved): the internal Humble backup libs
would not load in this install, and `setup_ros_env.bat` only appends the lib path when
`ROS_DISTRO` is *unset* (presetting it skips the PATH block entirely).
**Fix:** `run_scene.bat` — a wrapper that `call`s `setup_ros_env.bat` (which selects the
default internal distro **jazzy** and wires its lib path correctly), sets the DDS env, then
runs `python.bat %*`. Jazzy-on-Isaac ↔ Humble-on-WSL interoperates fine for the message types
used (RTPS is the wire protocol; both are Fast DDS).

### Issue 8 — OmniGraph publishers need a `ROS2Context` node

**Symptom:** with the bridge finally loaded, the graph built but publishers errored:
`Unable to create ROS2 node, please check that namespace is valid`.
**Diagnosis:** compared against the shipped `moveit.py` example: every `ROS2Publish*` node
needs a **`isaacsim.ros2.bridge.ROS2Context`** node with `Context.outputs:context →
Publish*.inputs:context`. Also `IsaacComputeTransformTree.inputs:targetPrims` expects
`[usdrt.Sdf.Path(prim_path)]`, not a bare string.
**Lesson:** on Isaac 6.0.1 (newer than any training data), the **shipped
`standalone_examples/` are the API ground truth** — every node type, attribute, and wiring
pattern used in `thin_slice.py` was lifted from `clock.py`, `moveit.py`,
`add_cubes.py`, and `test_publish_camera_data.py` rather than guessed.

### Issue 9 — THE core issue: everything runs, data still doesn't cross

**Symptom:** bridge loaded, graph built, publishers created without error, scene visibly
running (cube moving in the GUI; `kit.exe` CPU advancing) — and WSL still received nothing,
discovery server or not.
**Isolation steps (each ruling something out):**
1. WSL-local pub→sub *through the discovery server* → **worked** (server fine, WSL fine).
2. Raw `rclpy` subscriber (no daemon, no CLI) for `/clock` → 0 messages (not a CLI artifact).
3. Plain-socket UDP test over `127.0.0.1`, both directions (Python listener/sender on WSL,
   `System.Net.Sockets.UdpClient` on Windows) → **both directions cross.** The road existed.
**Diagnosis:** with the physical path proven and both ROS 2 stacks healthy, the failure had to
be *which transport/locators DDS chose*:
- **Shared-memory trap:** both OSes report IP `10.60.166.51`, so Fast DDS classifies the peer
  as *same-host* and prefers **SHM transport**. Windows SHM segments and Linux SHM segments
  are different universes → data black-holes after successful discovery.
- **Locator trap:** each participant advertises its interface IP (`10.60.166.51`) as its data
  locator. A WSL packet sent to its *own* IP loops back into WSL; it never crosses.
- **(Latent) protocol trap:** Fast DDS 2.6 (Humble) ↔ 3.x (Isaac jazzy) discovery-server
  compatibility is not guaranteed across the major version.
**Fix — `config/fastdds-loopback.xml`, identical on both sides:**
```xml
<transport_descriptor>            <!-- UDPv4 only; declared transport replaces built-ins -->
  <transport_id>UdpLoopbackOnly</transport_id>
  <type>UDPv4</type>
  <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
</transport_descriptor>
...
<rtps>
  <useBuiltinTransports>false</useBuiltinTransports>   <!-- kills SHM -->
  <userTransports><transport_id>UdpLoopbackOnly</transport_id></userTransports>
  <builtin>
    <initialPeersList><locator><udpv4><address>127.0.0.1</address></udpv4></locator></initialPeersList>
  </builtin>
</rtps>
```
- `useBuiltinTransports=false` + UDP-only → **no SHM**, ever.
- `interfaceWhiteList 127.0.0.1` → every advertised locator is loopback, the one address
  proven to cross both ways.
- `initialPeersList 127.0.0.1` → participants send periodic unicast announcements to the
  default DDS ports on loopback (covering participant indices 0..4), so they find each other
  with **no multicast and no discovery server** — plain RTPS peer discovery, version-agnostic.
Env hookup: WSL `FASTRTPS_DEFAULT_PROFILES_FILE=<repo>/config/fastdds-loopback.xml`
(`ros-env.sh`); Windows sets **both** `FASTRTPS_DEFAULT_PROFILES_FILE` and
`FASTDDS_DEFAULT_PROFILES_FILE` (the variable was renamed in Fast DDS 3.x) in
`run_scene.bat`, plus `set ROS_DISCOVERY_SERVER=` to clear the stale `setx` value.
**Verification:** rclpy probes received `/clock` (sim time advancing at 60 Hz steps) and `/tf`
`world→target` with position advancing at exactly the commanded (0.5, 0.2, 0.0) m/s.

### Issue 10 — Miscellaneous gotchas (each cost real time)

- **Kit stdout is block-buffered.** The launch log "freezes" at `app ready` while the scene
  runs fine; Python prints appear only on exit/flush. Never diagnose liveness from the log —
  check `kit.exe` CPU delta (`Get-Process kit | % CPU` twice) or observe the GUI.
- **Stale console handles lock log files.** A dead run's console kept `thin_slice.log` open →
  next launch failed with *"The process cannot access the file because it is being used by
  another process."* Use a fresh log filename per attempt.
- **`ros2 topic echo` can be silent where rclpy receives.** The `/tf` stream echoed nothing
  via CLI but a 10-line rclpy subscriber got it immediately. Trust rclpy probes for
  ground truth; the CLI adds daemon/QoS layers that can fail independently.
- **`sequence size exceeds remaining buffer` warnings** from the ros2 CLI in this setup are
  cosmetic; payloads decode fine.
- **GUI Pause/Stop doesn't stop the cube** in `thin_slice.py`: the Python while-loop drives
  `set_world_poses` independent of the timeline. Cosmetic for the thin slice; the proper fix
  (later) is driving motion from the physics/playback tick.
- **Killing Isaac:** `taskkill /F /T /IM kit.exe` (or Task Manager → `kit.exe`). `kit.exe` is
  both the GUI app and `python.bat`'s interpreter (python.exe is copied to kit.exe for driver
  profile reasons), so this kills standalone scenes too. `Stop-Process` alone can leave the
  tree half-alive mid-shutdown.
- **Windows env vars via `setx` only reach *new* processes.** Isaac must be fully relaunched
  (not just scene-reloaded) to see them.
- **zsh vs bash:** the project's shell is zsh; `source /opt/ros/humble/setup.bash` fails under
  zsh. `ros-env.sh` branches on `$ZSH_VERSION` → `setup.zsh` / `setup.bash`.
- **This sandbox blocks foreground `sleep`** (exit 144) and gets flaky with `pkill` in
  compound commands — long-running/waiting work goes in background tasks instead.

---

## 2b. Epilogue — first live L1 run over the bridge (same day)

With the bridge working, the thin-slice tracker node (`src/mini_lattice/edge/thin_slice_node.py`)
consumed `/tf`, synthesized camera+radar measurements through the real observation models, and
ran the 9-D UKF. Two more issues surfaced immediately — estimation bugs, not bridge bugs, but
part of the same day's record:

- **`PYTHONPATH=src` clobbered rclpy.** `setup.bash` delivers rclpy via `PYTHONPATH`;
  overwriting it (instead of prepending `src:$PYTHONPATH`) made `import rclpy` fail.
- **UKF crashed non-positive-definite on the first camera update.** Three stacked causes:
  (1) standard small-alpha sigma weights are catastrophically negative at n=9 (α=1e-3 →
  center weights ±1e6) — fixed with α=1, κ=1 (all weights positive → PSD by construction);
  (2) zero-velocity init + the velocity→yaw→box-width coupling made sigma points span all
  headings and the camera update overshoot — fixed with two-point-differencing velocity init;
  (3) update ordering — radar (mild nonlinearity) now precedes camera (harsh). Plus defensive
  symmetrization after predict/update and an eigenvalue-floor repair in sigma-point
  generation. Full detail: `explanations/edge/ukf.md` ("War story").

**Final verified result:** 238 consecutive fusion ticks, position error 2–13 cm at ~400 m
range, velocity ≈ truth, extent converging 0.5 → 0.33 toward the 0.2 truth. The complete
`Isaac scene → ROS 2 across the WSL boundary → 9-D camera+radar UKF fusion` slice works.

## 2c. Issue 11 (2026-07-30) — OmniGraph TF branch silently dies; rclpy direct publish

**Symptom:** across relaunches of the same scene, `/clock` published forever but `/tf`
published only for the first seconds (or a longer window — nondeterministic), then went
silent. No errors in the log.
**Diagnosis:** OmniGraph **disables a node that errors during execution** — the
`IsaacComputeTransformTree → ROS2PublishTransformTree` branch died internally (likely racing
the scripted `set_world_poses` writes) while the independent clock branch kept ticking.
A disabled branch produces no diagnostics at the ROS level: the classic "silent partial
failure."
**Fix (structural, not a retry):** publish the ground-truth pose **directly from the scene's
Python loop via Isaac's internal rclpy** (`geometry_msgs/PoseStamped` on `/target_pose`,
stamped with sim time) — the pattern the shipped `clock.py` example uses. No OmniGraph in
the data path. Verified: 683 consecutive tracker cycles over 150 s with zero dropouts.
**Rule:** for scripted scenes, prefer in-scene rclpy publishing for data you compute in the
script; use OmniGraph publishers for data OmniGraph itself computes (sensors, clock).

## 2d. Issue 12 (2026-07-30) — port-less initial peers cap discovery at 5 participants

**Symptom:** with six+ ROS 2 nodes running, a newly launched node (the L4 gate) was
undiscoverable — a publisher waiting on `get_subscription_count()` saw 0 subscribers forever.
**Diagnosis:** a port-less `<initialPeersList>` locator probes the well-known metatraffic
ports only for **participant ids 0–4**. Busy graphs push new participants past index 4;
nobody ever knocks on their ports.
**Fix:** explicit peer locators for metatraffic ports `7410 + 2*id`, ids 0–19, in
`config/fastdds-loopback.xml` (both sides). Verified immediately: the same publisher matched
1 subscriber and the message flowed.
**Rule:** in unicast-initial-peers setups, size the peer port list to the node population.

## 3. Theory recap (the interview-grade version)

- **DDS discovery** has three modes: **multicast** (default; dead across the WSL boundary),
  **discovery server** (unicast hub; crossed the boundary but version-fragile and ultimately
  unnecessary), and **initial peers** (direct unicast announcements; what we use — simplest
  and version-agnostic).
- **Discovery ≠ data.** Fast DDS negotiates *transports and locators* per matched peer after
  discovery. Matching can succeed while data goes to an unusable transport (SHM) or an
  unroutable locator (own-IP). This is why "topics visible but no messages" is a *transport*
  symptom, not a discovery symptom.
- **Shared-memory transport** is Fast DDS's same-host fast path, selected on IP identity.
  Mirrored WSL breaks the "same IP ⇒ same OS" assumption — the defining trap of this setup.
- **Locators** are the addresses a participant advertises for reaching it. Interface
  whitelisting is the lever that controls what gets advertised.
- **Debugging ladder that worked:** prove the physical path with raw sockets → prove each side
  locally → then question the middleware's transport selection. Instrument *under* the failing
  layer, not on top of it.
