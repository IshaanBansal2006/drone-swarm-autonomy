# 030/031 — COP viewer: Rerun; transport: Rerun SDK for viz + ROS 2 inter-layer

**Status:** Accepted
**Date:** 2026-07-30

## Context

L3 (Step 2) needs the operator's 3-D common operating picture: drones, tracks, sensor
coverage, mission state, live. Forks L3-a (viewer) and L3-b (transport).

## Options considered (L3-a viewer)

- **Three.js + React** — user knows it (APE_GCS); full bespoke control; most build effort
  (every scene primitive hand-rolled).
- **Rerun** *(chosen)* — purpose-built robotics/physical-AI visualization: 3-D scenes,
  time-scrubbing, entity paths, tensors/images, Python SDK.
- **Foxglove Studio** — ROS-native, panel ecosystem; heavier, less bespoke.
- **Unity + WebGL** — prettiest demos; largest learning curve; wrong effort allocation.

## Options considered (L3-b transport)

WebSocket-JSON / WebSocket-protobuf / gRPC-web / rosbridge — all presumed a hand-built
browser frontend. Choosing Rerun collapses most of this fork: the viz feed goes through the
**Rerun SDK** (its own efficient transport), and **ROS 2 remains the inter-layer transport**
(040 schemas) — Rerun taps the same feeds the layers already exchange.

## Decision

**Rerun for the COP picture** (user: "good to use something made for physical AI" — tooling
signal aligned with the roadmap's target labs, and weeks of Three.js scene plumbing saved).
**Transport: Rerun SDK for visualization; ROS 2 + 040 schemas between layers (unchanged).**

**Honest limitation, recorded:** Rerun is a *viewer*, not an interactive control surface —
it does not host custom buttons/forms. The OPERATOR INPUT path (issuing StructuredIntents,
L4 approve/deny) needs its own channel: Step 2 starts with a terminal/CLI operator console;
a small web panel can complement later if demo polish demands it. If the COP ever needs to
be a fully bespoke interactive terminal, Three.js remains the documented fallback (schemas
make the viz layer swappable).

## Consequences

- `cop/` gets a Rerun bridge node: subscribes `/tracks` (+ later drone/mission topics),
  logs entities (track boxes with class labels + Bel/Pl, drone poses, sensor cones, patrol
  routes) to a Rerun viewer session.
- `rerun-sdk` joins project dependencies (WSL side; viewer window runs on Windows or WSLg).
- L4's approve/deny starts as a CLI prompt tool speaking `EngagementProposal` over ROS 2.
