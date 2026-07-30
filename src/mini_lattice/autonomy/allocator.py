"""CBBA task allocation (decision 021 — Choi, Brunet & How 2009).

Two alternating phases per round:
  1. BUNDLE BUILD (greedy, per drone): repeatedly add the task with the best
     *marginal* score improvement at its best path-insertion point, but only if
     that score would OUTBID the task's current global winner.
  2. CONSENSUS: drones exchange (bids, winners); for each task the highest bid
     wins (ties -> lower drone_id). A drone that loses a task drops it AND every
     task it added later (bundle truncation — later additions were scored
     assuming the lost task was in the path, so their bids are stale).

Rounds repeat until nothing changes. Scores are time-discounted rewards
S = lambda^tau * reward (tau = arrival time along the drone's path), which are
diminishing-marginal-gain — the property CBBA's convergence guarantee needs.

Documented simplification (backlog): fully-connected synchronous communication,
so consensus is a global max per task each round. The full Choi conflict-
resolution table (36 rules with timestamps) is only needed for multi-hop /
asynchronous networks — it slots in behind the same interface when comms get
realistic. Convergence bound: <= N_tasks rounds under full connectivity.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from mini_lattice.autonomy.decomposer import Task
from mini_lattice.autonomy.world_state import DroneState

LAMBDA = 0.95  # time discount per second of travel; sets urgency of proximity
MAX_BUNDLE = 5  # tasks per drone per allocation round (compute guard)


@dataclass
class _Agent:
    """Per-drone CBBA working state."""

    drone: DroneState
    path: list[Task] = field(default_factory=list)  # ordered execution route


def _task_entry(task: Task) -> np.ndarray:
    return np.asarray(task.waypoints[0] if task.waypoints else [0.0, 0.0, 0.0])


def _task_exit(task: Task) -> np.ndarray:
    return np.asarray(task.waypoints[-1] if task.waypoints else [0.0, 0.0, 0.0])


def _path_score(agent: _Agent, path: list[Task]) -> float:
    """Sum of lambda^arrival_time * reward along the path (straight-line legs)."""
    pos = np.asarray(agent.drone.position)
    t = 0.0
    score = 0.0
    for task in path:
        t += float(np.linalg.norm(_task_entry(task) - pos)) / agent.drone.speed
        score += (LAMBDA ** t) * task.reward
        # traverse the task's own waypoints before heading onward
        wps = [np.asarray(w) for w in task.waypoints]
        for a, b in zip(wps, wps[1:]):
            t += float(np.linalg.norm(b - a)) / agent.drone.speed
        pos = _task_exit(task)
    return score


def _capable(drone: DroneState, task: Task) -> bool:
    return task.required_capability is None or task.required_capability in drone.capabilities


class CBBAAllocator:
    def allocate(self, tasks: list[Task], drones: list[DroneState],
                 max_rounds: int | None = None) -> dict[str, list[Task]]:
        """Returns drone_id -> ordered task path. Unallocatable tasks are absent
        (no capable drone, or bundles full) — caller decides whether that's an
        error or a next-round situation."""
        agents = {d.drone_id: _Agent(drone=d) for d in drones if d.available}
        if not agents or not tasks:
            return {a: [] for a in agents}
        bids: dict[str, float] = {t.task_id: 0.0 for t in tasks}
        winners: dict[str, str | None] = {t.task_id: None for t in tasks}
        by_id = {t.task_id: t for t in tasks}

        for _ in range(max_rounds or len(tasks) + 1):
            changed = False
            # -- phase 1: bundle building --------------------------------
            for aid in sorted(agents):
                agent = agents[aid]
                while len(agent.path) < MAX_BUNDLE:
                    best = None  # (marginal, insert_pos, task)
                    base = _path_score(agent, agent.path)
                    for task in tasks:
                        if task in agent.path or not _capable(agent.drone, task):
                            continue
                        for pos in range(len(agent.path) + 1):
                            trial = agent.path[:pos] + [task] + agent.path[pos:]
                            marginal = _path_score(agent, trial) - base
                            if marginal > bids[task.task_id] + 1e-12 and (
                                best is None or marginal > best[0]
                            ):
                                best = (marginal, pos, task)
                    if best is None:
                        break
                    marginal, pos, task = best
                    agent.path.insert(pos, task)
                    bids[task.task_id] = marginal
                    winners[task.task_id] = aid
                    changed = True
            # -- phase 2: consensus (fully connected: global max wins) ---
            for aid in sorted(agents):
                agent = agents[aid]
                for i, task in enumerate(agent.path):
                    if winners[task.task_id] != aid:
                        # lost this task -> drop it and everything added after
                        for dropped in agent.path[i:]:
                            if winners[dropped.task_id] == aid:
                                bids[dropped.task_id] = 0.0
                                winners[dropped.task_id] = None
                        agent.path = agent.path[:i]
                        changed = True
                        break
            if not changed:
                break

        return {aid: list(agents[aid].path) for aid in agents}
