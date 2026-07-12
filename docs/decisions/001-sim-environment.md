# 001 — Sim environment

**Status:** Accepted
**Date:** 2026-07-11

## Context

Mini-Lattice needs a simulation environment to host N drones + M sensors before any real-hardware step. The choice constrains sensor realism, iteration speed, industry-tool signal, sim-to-real path viability, and cross-boundary transport (this dev machine is WSL2 Ubuntu 20.04 on top of Windows 11 with an RTX 4070 Laptop, 8GB VRAM, 32GB system RAM).

## Options considered

**Sim environment (D1):**
- **A — gym-pybullet-drones:** fast Python drone RL sim. No sensor sim. Weak industry signal.
- **B — Isaac Sim:** Nvidia's flagship robotics simulator. Photoreal radar/camera/IR. Strongest industry signal (Nvidia GEAR / Anduril / Skydio / DeepMind Robotics use it). Highest GPU/RAM cost. 2–3 week learning curve.
- **C — Gazebo + PX4 SITL:** older robotics sim + real PX4 firmware. Strong hardware-stack signal. Slower iteration; less photoreal.
- **D — Custom minimal sim:** total control, weeks of infrastructure work not tied to core project.
- **E — Hybrid (pybullet + fake sensor emulator):** fast + you own sensor semantics; weak tool signal.

**Runtime location (given Isaac Sim):**
- **W1 — Isaac Sim on Windows host, code in WSL, ROS2 across the boundary:** clean separation; Windows gets full GPU driver stack; WSL keeps the code/dev loop; sim-to-real path becomes a driver swap.
- **W2 — Isaac Sim on Windows, code in WSL, WebSocket bridge:** simpler bridge; not ROS-native; real-hardware transition requires rewriting the ingest layer.
- **W3 — Isaac Sim inside WSL2:** single environment; requires bumping WSL memory to ~24GB; WSLg viewport latency; Linux Isaac Sim lags Windows by ~1 release.

## Decision

**Isaac Sim on Windows 11 host + ROS2 across the WSL2 boundary (W1).**

## Consequences

- **Sim and code live in different environments.** Fast DDS discovery must be configured so Windows-side ROS2 and WSL-side ROS2 see each other on the WSL virtual NIC. Add a Windows firewall exception for the DDS discovery + user-data ports.
- **L1 / L2 / L3 code is ROS2-native.** Message layer between sim and L1 is ROS2 messages.
- **Decision `031` (data transport) is partially resolved.** ROS2 covers the sim-boundary + L1/L2 comms. The browser-facing COP frontend still needs its own transport decision (WebSocket, Foxglove, Rerun).
- **Decision `040` (interface schemas) is now scoped to ROS2 `.msg` / IDL types** (or pydantic models that serialize to ROS2 messages).
- **Real-hardware stretch (`006`, D6) becomes cheap.** Replace Isaac Sim publishers with real sensor/state drivers on the same ROS2 topics; L1/L2/L3 code is unchanged. Isaac-to-real is now a driver swap, not a rewrite.
- **Ties into existing `~/px4_ros2_ws/`** (ROS2 Foxy + `px4_ros_com` + uXRCE-DDS). Reuse this workspace.
- **VRAM ceiling acknowledged.** RTX 4070 Laptop 8GB VRAM is the minimum for Isaac Sim. Cap scene complexity at 2–3 drones + 2–4 sensors initially. If we hit VRAM ceilings, simplify materials/scene assets before considering cloud GPU (Lambda, RunPod, Nvidia LaunchPad).
- **D5 (intent-parser tech) is now constrained.** A local LLM on the same GPU as Isaac Sim will not fit alongside a moderate scene. This pushes D5 toward: API-based LLM (Claude/GPT), a CPU-quantized local model (ollama on CPU or `llama.cpp` w/ `-ngl 0`), or a DSL.
- **Learning-curve budget.** 2–3 weeks for Isaac Sim + ROS2 bridge setup before productive layer work. Bake into `002` (roadmap slot / timing).
- **Windows RAM headroom is tight.** 32GB total; Isaac Sim + Windows baseline + WSL cap will use most of it. Close Chrome/JetBrains before long runs.

## Setup checklist (execution notes; not part of the decision)

- [ ] Install Nvidia Omniverse Launcher on Windows
- [ ] Install Isaac Sim via Omniverse (latest stable)
- [ ] Enable Isaac Sim ROS2 Bridge extension (matches ROS2 Foxy on WSL)
- [ ] Configure Fast DDS profile (XML) on both sides so discovery works across the WSL vEthernet NIC
- [ ] Add Windows firewall exception for DDS discovery + user data ports (7400 base + offsets)
- [ ] Verify: WSL `ros2 topic list` sees a topic published by Isaac Sim on Windows
- [ ] Verify: WSL publishes a topic that Isaac Sim receives (cmd_vel or similar)
- [ ] Optional: install `ros1_bridge` if kanan / ArduPilot integration ever becomes relevant (later)
