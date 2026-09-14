"""L2 mission node — the classical brain, live (Step 1 of decision 008).

Wires the decided stack end-to-end as a ROS 2 node:

    /intent (StructuredIntent JSON)        [from cop/console.py]
    /tracks (L1 confirmed tracks JSON)     [from edge/thin_slice_node.py]
        -> WorldState -> HTN decompose (020, Voronoi scan via 022)
        -> CBBA allocate (021) -> Executor ticks BTs on the KinematicBackend
    /drone_poses (041 DronePoseFrame, truth) [Isaac renders these prims; L1 synthesises from them]
    /drone_pose_estimates (041, from L1)     -> DroneState.pose_estimate
    /mission_status (JSON)                 [for the Rerun COP bridge]
    /engagement_proposals (040 JSON) <-> /engagement_decisions   [L4 gate]

Replanning (design-doc L2 M6, minimal trigger set — documented):
  - a NEW intent arrives            -> full re-decompose + re-allocate
  - a follow_track task FAILS       -> its track vanished; re-allocate the rest
Full replans preempt everything (Executor.assign replaces queues): trees are
cheap; rebuilding beats patching (020's reactive stance).

Engagement demo rule: a follower closing within ENGAGE_RANGE of its target
emits ONE EngagementProposal ("designate"); execution of the action itself is
out of scope for Step 2 — the point is the L4 approval loop + audit trail.

Kinematics run HERE (single source of truth = WorldState); Isaac only renders
the commanded drone prims. This inverts when the DroneBackend upgrades to real
quadrotor dynamics (008 fidelity ladder): then the sim owns state and the
backend becomes a command bridge.
"""

from __future__ import annotations

import itertools
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pathlib import Path

from swarm_autonomy.autonomy.allocator import CBBAAllocator
from swarm_autonomy.autonomy.learned_score import LearnedScore
from swarm_autonomy.autonomy.coverage import VoronoiCoverage
from swarm_autonomy.autonomy.decomposer import HTNDecomposer, Task
from swarm_autonomy.autonomy.executor import Executor, SmoothBackend
from swarm_autonomy.autonomy.world_state import DroneState, WorldState
from swarm_autonomy.edge import rotation
from swarm_autonomy.scene import demo_scene
from swarm_autonomy.schemas import (
    DronePoseFrame,
    DronePoseMsg,
    EngagementProposal,
    StructuredIntent,
    TrackFrame,
)

TICK_DT = 0.1  # control cycle (s)
ENGAGE_RANGE = 3.0  # m — follower proximity that triggers a proposal

SCENE = demo_scene()
LEARNED_SCORE_PATH = Path(__file__).resolve().parents[3] / "config" / "learned_score.json"
FLEET = [  # start poses come from the shared scene (decision 019)
    DroneState(drone_id=did, position=list(start.position),
               orientation=rotation.from_yaw(start.yaw).tolist(),
               capabilities=frozenset({"camera"}))
    for did, start in SCENE.fleet.items()
]


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("l2_mission")
        self.world = WorldState()
        for d in FLEET:
            self.world.update_drone(d)
        self.decomposer = HTNDecomposer(coverage=VoronoiCoverage())
        weights = LEARNED_SCORE_PATH
        if weights.exists():  # decision 024: localization-aware score when trained
            self.allocator = CBBAAllocator(score=LearnedScore.from_json(weights, SCENE.signs))
            self.get_logger().info(f"CBBA scoring with learned score {weights}")
        else:
            self.allocator = CBBAAllocator()
            self.get_logger().info("CBBA scoring with the time-discounted baseline")
        self.backend = SmoothBackend(self.world)
        self.task_executor = Executor(self.backend, self.world)

        self.intents: dict[str, StructuredIntent] = {}
        self.tasks: dict[str, Task] = {}  # unfinished, by task_id
        self._proposed_tracks: set[int] = set()
        self._proposal_ids = itertools.count()
        self._approved: set[str] = set()

        self.create_subscription(String, "/intent", self._on_intent, 10)
        self.create_subscription(String, "/tracks", self._on_tracks, 10)
        self.create_subscription(String, "/engagement_decisions", self._on_decision, 10)
        self.create_subscription(String, "/drone_pose_estimates", self._on_pose_estimates, 10)
        self.pub_poses = self.create_publisher(String, "/drone_poses", 10)
        self.pub_status = self.create_publisher(String, "/mission_status", 10)
        self.pub_proposals = self.create_publisher(String, "/engagement_proposals", 10)
        self.create_timer(TICK_DT, self._tick)
        self.get_logger().info("L2 mission node up: %d drones" % len(FLEET))

    # ------------------------------------------------------------- inputs
    def _on_intent(self, msg: String) -> None:
        intent = StructuredIntent.model_validate_json(msg.data)
        self.intents[intent.intent_id] = intent
        self.get_logger().info(f"intent {intent.intent_id}: {intent.verb}")
        self._replan(f"new intent {intent.intent_id}")

    def _on_tracks(self, msg: String) -> None:
        # Validated 040 TrackFrame — no hand-adapted keys. A producer/consumer
        # field mismatch now fails HERE, loudly, at the boundary.
        frame = TrackFrame.model_validate_json(msg.data)
        self.world.update_tracks(frame.tracks, frame.timestamp)

    def _on_pose_estimates(self, msg: String) -> None:
        frame = DronePoseFrame.model_validate_json(msg.data)
        unknown = {p.drone_id for p in frame.poses} - set(self.world.drones)
        if unknown:
            self.get_logger().warning(
                f"pose estimates for unknown drones {sorted(unknown)} ignored — "
                f"L1 and L2 fleet definitions disagree")
        self.world.update_pose_estimates(frame)

    def _on_decision(self, msg: String) -> None:
        d = json.loads(msg.data)
        if d.get("approved"):
            self._approved.add(d["proposal_id"])
        self.get_logger().info(
            f"L4 decision: {d['proposal_id']} "
            f"{'APPROVED' if d.get('approved') else 'DENIED'}")

    # ------------------------------------------------------------- planning
    def _replan(self, reason: str) -> None:
        """Full re-decompose + re-allocate of everything unfinished (see module
        docstring for the trigger philosophy)."""
        self.get_logger().info(f"replanning: {reason}")
        tasks: list[Task] = []
        for intent in self.intents.values():
            try:
                tasks.extend(self.decomposer.decompose(intent, self.world))
            except (ValueError, NotImplementedError) as e:
                self.get_logger().error(f"intent {intent.intent_id} dropped: {e}")
                continue
        done = set(self.task_executor.completed)
        tasks = [t for t in tasks if t.task_id not in done]
        self.tasks = {t.task_id: t for t in tasks}
        allocation = self.allocator.allocate(tasks, self.world.available_drones())
        self.task_executor.assign(allocation)

    # ------------------------------------------------------------- control
    def _tick(self) -> None:
        before_failed = len(self.task_executor.failed)
        self.task_executor.tick(TICK_DT)
        if len(self.task_executor.failed) > before_failed:
            self._replan("task failure (track lost?)")
        self._maybe_propose()
        t = self.get_clock().now().nanoseconds * 1e-9
        self.pub_poses.publish(String(data=DronePoseFrame(timestamp=t, poses=[
            DronePoseMsg(drone_id=d.drone_id, timestamp=t, position=list(d.position),
                         orientation=list(d.orientation))
            for d in self.world.drones.values()]).model_dump_json()))
        self.pub_status.publish(String(data=json.dumps({
            "intents": list(self.intents),
            "pending_tasks": len(self.tasks) - len(self.task_executor.completed),
            "completed": self.task_executor.completed[-5:],
            "failed": self.task_executor.failed[-5:],
        })))

    def _maybe_propose(self) -> None:
        """Demo engagement rule: follower within ENGAGE_RANGE -> one proposal."""
        for q_drone, queue in self.task_executor._queues.items():
            task = queue.current
            if task is None or task.task_type != "follow_track":
                continue
            track = self.world.tracks.get(task.target_track_id)
            if track is None or track.track_id in self._proposed_tracks:
                continue
            dist = float(np.linalg.norm(
                np.asarray(self.world.drones[q_drone].position)
                - np.asarray(track.position)))
            if dist < ENGAGE_RANGE:
                self._proposed_tracks.add(track.track_id)
                p = EngagementProposal(
                    proposal_id=f"p{next(self._proposal_ids)}",
                    action="designate", target_track_id=track.track_id,
                    rationale=f"{q_drone} holding {dist:.1f} m from track "
                              f"{track.track_id} ({track.class_label})",
                    deadline_s=15.0)
                self.pub_proposals.publish(String(data=p.model_dump_json()))
                self.get_logger().info(f"engagement proposed: {p.proposal_id}")


def main() -> None:  # pragma: no cover
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
