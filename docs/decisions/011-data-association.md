# 011 — Data association

**Status:** Accepted
**Date:** 2026-07-13

## Context

Given N existing tracks (maintained by UKF, per decision 010) and M new measurements from heterogeneous sensors (decision 007), L1 must determine which measurements belong to which tracks, which are new targets, and which are false alarms. This is the association problem. It is tightly coupled with the state estimator — the filter's predicted measurement and innovation covariance define the gating and distance metric.

## Options considered

- **GNN (Global Nearest Neighbor via Hungarian):** optimal hard assignment in O(n³). Simple. Fails when targets are close or clutter is dense — one wrong assignment corrupts the track.
- **JPDA (Joint Probabilistic Data Association):** soft assignments — each measurement contributes to each track with a probability weight. Handles clutter and nearby targets. Combinatorial in the worst case but tractable with gating. Memoryless (no hypothesis history across time steps). Known failure mode: identity merge on crossing targets.
- **MHT (Multiple Hypothesis Tracking):** maintains hypothesis tree over time. Defers hard decisions. Gold standard for defense tracking. Most complex to implement. Exponential growth requires pruning (N-scan, Murty's k-best).
- **Learned association:** transformer attention or learned cost matrix. Needs training data. Black box. Not production-standard in defense.

## Decision

**JPDA first, with MHT as a fallback if JPDA's limitations become blocking.**

JPDA's soft association handles the expected sim complexity (2–3 drones, 5–10 targets, moderate clutter). Its memoryless nature and crossing-target merge failure mode are known — if the sim scenarios expose these (dense target crossings, prolonged close proximity), upgrade to MHT.

The upgrade path is clean: MHT replaces the association layer while the UKF filter and observation models remain unchanged. JPDA → MHT is a swap at the association interface, not a rewrite.

## Consequences

- **User implements:** gating (Mahalanobis distance + chi-squared threshold), feasible association event enumeration, association probability computation, combined innovation for UKF update.
- **Gating is shared.** Both JPDA and MHT use the same Mahalanobis gating logic — implement it as a reusable component.
- **Track management (initiation, deletion, merge) sits alongside association.** JPDA does not natively handle track birth/death — add an M/N logic layer (M detections in N scans → initiate track; K missed updates → delete).
- **Cross-modal association.** Measurements from different sensor types live in different observation spaces. Gate using the UKF's predicted measurement in each sensor's space (the UKF handles the nonlinear transform per sensor). Association probabilities are computed in innovation space, which is sensor-specific — weight by sensor reliability or measurement likelihood.
- **MHT upgrade trigger:** if sim testing shows persistent identity switches on crossing targets or unacceptable track fragmentation in clutter, switch to MHT. Document the failure cases as evidence for the switch.
