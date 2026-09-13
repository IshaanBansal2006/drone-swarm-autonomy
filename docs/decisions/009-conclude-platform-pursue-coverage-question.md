# 009 — Conclude the platform, pursue the coverage question

**Status:** Accepted
**Date:** 2026-07-31 (recorded 2026-09-13)
**Supersedes:** `008` (project vision and staged build plan), steps 3–5
**Amended by:** `050` — the repository reopened on 2026-09-13 for two capability extensions; the
research pivot recorded here stands

## Context

`008` laid out a six-step plan: build the classical four-layer stack (steps 0–2), then add a
language front-end (step 3), a learned control baseline (step 4), and a vision-language-action
layer (step 5) — three interchangeable brains behind one L2 interface, so the deltas between them
attribute value to learning and then to pretrained grounding.

Steps 0–2 were built and demonstrated end to end. Working through what step 5 would actually
require produced a sharper question than the platform it was meant to sit inside:

> Classical coverage control moves a swarm to match an **importance map**, and that map is
> conventionally hand-specified. A vision-language model can generate it from what the drones see
> plus a natural-language mission. But then the map that guides the swarm is also a map that can be
> wrong, shared over an unreliable radio, and corrupted in a way that misdirects the very
> observations that would correct it.

That failure loop is measurable, and measuring it is a contribution. The surrounding C2 platform is
not — and the pipeline the question needs contains no multi-object tracking, no task auction, and
no approval gate, which is most of what steps 0–2 built.

The ablation ladder in `008` also stopped justifying itself. Its three-brain comparison is only
valid if all three share the same tasks, metrics and action interface; once the interesting task
became *semantic coverage under fault*, the classical brain was no longer a baseline for the same
problem, so the deltas would have measured nothing.

## Options considered

**A. Continue steps 3–5 in this repository, as `008` planned.**
- *Pro:* one continuous artifact; the swappable-brain seam already exists; no duplicated code.
- *Con:* ~80% of the shipped code (tracking stack, CBBA, HTN/BT, approval gate) is dead weight for
  the new question — carried forward unused, or deleted, which destroys a finished and tested
  artifact.
- *Con:* the platform's value is that it is fully public — every decision, both post-mortems, an
  honest backlog. A live research repo needs the opposite posture while results are unsettled. One
  repository cannot run both policies.
- *Con:* the experiment's history buries itself under platform commits; "done" means something
  different for each (*runs end to end* vs. *produces a measured curve*).

**B. Strip this repository down to the reusable seam and rebuild in place.**
- *Pro:* no duplication; keeps one URL and one commit history.
- *Con:* rewrites a demonstrated system into a half-empty one. The four `video-*` tags stop
  describing the repository that contains them, and the deep-dive documents code that is gone.
- *Con:* the strongest thing the platform has to show — that it was finished — is destroyed by the
  commit that starts the next project.

**C. Conclude this repository; start a focused one carrying the seam forward.** ← chosen
- *Pro:* the platform stays a complete, tested, documented artifact with its limits stated plainly.
- *Pro:* the research repo starts with the right scope, the right privacy posture, and a commit
  history that is about the experiment.
- *Con:* the shared seam is **copied, not depended on**, so the two copies can drift.

## Decision

**Close this repository to new feature work at step 2 and continue as a separate, focused project —
[`vlm-swarm-coverage`](https://github.com/IshaanBansal2006/vlm-swarm-coverage) — carrying forward
only the infrastructure the new question needs.** Steps 3–5 of `008` are withdrawn rather than
deferred.

**Carried forward:** the Isaac↔WSL bridge recipe and `config/fastdds-loopback.xml`, the multi-drone
scene and the `DroneBackend` seam, the `CoveragePlanner` protocol, the Rerun operating-picture
pattern, the typed-schema discipline, and the conflict-weighted Dempster–Shafer fusion from `015`
as the new project's Tier-2 mechanism.

**Not carried forward:** the tracking stack (SR-UKF, PDA, track lifecycle, classification plumbing),
CBBA, HTN/BT execution, and the L4 approval gate.

## Consequences

- `008` is superseded from step 3 onward. Steps 0–2 remain an accurate record of what was built;
  its steps 3–5, its three-brain ablation, its dynamics-fidelity ladder past kinematics, and its
  compute/data plan describe a project that will not happen here.
- The open items in `docs/backlog.md` are **not a roadmap**. They stay because they honestly
  describe the system's limits; the pivot note at the end of that file says so explicitly.
- Work stopped mid-flight is recorded where it lies rather than deleted: joint-event enumeration in
  `edge/jpda.py` and the hand-written Hungarian in `assignment.py` are implemented and tested but
  never integrated — see `016`'s stop note.
- The hardware track (`008`'s later steps, future `006-real-hardware.md`, D-B8 in the backlog) is
  withdrawn with steps 3–5. It was contingent on a learned brain that is not being built here.
- The seam is duplicated across two repositories. **Accepted, not solved.** Extracting it into a
  shared installable package is not worth the overhead while one side is frozen; the trigger to
  revisit is fixing the same bridge or DDS bug in both places.
- This repository accepts issues and pull requests but makes no commitment to answer them.
