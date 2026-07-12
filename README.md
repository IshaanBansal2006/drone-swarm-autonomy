# Mini-Lattice

Multi-agent command-and-control platform. One operator, N simulated drones + heterogeneous sensors. Inspired by Anduril's Lattice OS.

## Layers

- **L1 — Distributed Edge** (sensor fusion) — deep, interview-defensible
- **L2 — Mission Autonomy** (intent → task → allocation) — deep, interview-defensible
- **L3 — Common Operating Picture** (COP) — moderate, demo-critical
- **L4 — Human-on-the-Loop** — light, safety story

## Design docs

- Full project guide: [Notion — Mini-Lattice](https://app.notion.com/p/399ca937182781478f0ccef0ac8be7a4)
- Obsidian: `~/Obsidian Vault/robot-learning-roadmap/mini-lattice-project.md`
- Architecture: [`docs/architecture.md`](docs/architecture.md)
- Decisions: [`docs/decisions/`](docs/decisions/)

## Status

Scoped. Open decisions in the project guide (sim env, timing, venue, category structure, intent-parser tech, real hardware). No code until each decision has a doc in `docs/decisions/`.

## Layout

```
mini_lattice/
  edge/       # L1 — sensor sim, fusion, MOT
  autonomy/   # L2 — intent parser, decomposer, allocator, replanner
  cop/        # L3 — operator terminal (backend for now; frontend elsewhere)
  hol/        # L4 — approve-to-engage gate
```

## Dev setup

```bash
uv venv
source .venv/bin/activate
uv pip install -e ".[dev]"
pytest
```

## Owner

Ishaan Bansal — Robot Learning Roadmap Path B.
