"""Live L1 node: Isaac truth -> synthesised IMU + detections -> SLAMMOT -> /tracks, /drone_pose_estimates.

Rewritten 2026-09-13 for decisions 017-019. The node's harness role is still
measurement synthesis, now for sensors that ride the drones:

    /target_pose (Isaac truth)   -> finite-difference velocity -> 9-D target truth
    /drone_poses (L2 truth, 041) -> per-drone IMU synthesis (at the arrival rate)
                                    + mounted-camera boxes of target / signs /
                                      parked vehicles + ground-radar returns
        -> SlammotPipeline: ego filters (IMU + sign fixes + vehicle landmarks)
                            and the consider tracker
        -> /tracks (040 TrackFrame)  /drone_pose_estimates (041 DronePoseFrame)

The tracker and the ego filters never see a true drone pose: L1 estimates it.
The IMU rate equals whatever rate /drone_poses arrives at (L2 ticks at 10 Hz),
so the live IMU is coarser than the 100 Hz simulator-free harness; the filter
is rate-agnostic (ImuConfig.rate_hz is set from the observed interval).

Run (WSL, Isaac scene + L2 mission node running):
    source scripts/ros-env.sh
    PYTHONPATH="src:$PYTHONPATH" python3 src/swarm_autonomy/edge/thin_slice_node.py --duration 40
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from swarm_autonomy.edge import rotation
from swarm_autonomy.edge.classification import DSClassifier
from swarm_autonomy.edge.config import EgoConfig, ImuConfig
from swarm_autonomy.edge.filters import sqrt_cov_block
from swarm_autonomy.edge.imu import ImuSynthesizer
from swarm_autonomy.edge.pipeline import PlatformInit, SlammotPipeline, road_config
from swarm_autonomy.edge.sensing import MeasurementSynthesizer, PlatformTruth, TargetTruth
from swarm_autonomy.scene import demo_scene
from swarm_autonomy.schemas import DronePoseFrame, TrackFrame, TrackMsg, encode_mass

MEAS_DT = 0.1  # fusion cadence; must equal TrackerConfig.ukf.dt
LIVE_IMU_HZ = 10.0  # /drone_poses arrives at L2's tick rate


class SlammotNode(Node):
    def __init__(self) -> None:
        super().__init__("l1_slammot")
        self.scene = demo_scene()
        self.drone_ids = sorted(self.scene.fleet)
        self.cfg = road_config(self.scene, self.drone_ids)
        self.imu_cfg = ImuConfig(rate_hz=LIVE_IMU_HZ)
        fleet = {did: PlatformInit(np.asarray(s.position, dtype=float), np.zeros(3),
                                   rotation.from_yaw(s.yaw))
                 for did, s in self.scene.fleet.items()}
        self.pipeline = SlammotPipeline(self.cfg, EgoConfig(imu=self.imu_cfg), self.scene, fleet)
        self.synth = MeasurementSynthesizer(self.scene, self.cfg, np.random.default_rng(42))
        self.imus = {did: ImuSynthesizer(self.imu_cfg, np.random.default_rng(i + 1))
                     for i, did in enumerate(self.drone_ids)}
        self.classifier = DSClassifier()
        self.tracks_pub = self.create_publisher(String, "/tracks", 10)
        self.poses_pub = self.create_publisher(String, "/drone_pose_estimates", 10)

        self._target: tuple[float, np.ndarray, np.ndarray | None] | None = None  # t, pos, vel
        self._platforms: dict[str, PlatformTruth] = {}
        self._prev_pos: dict[str, tuple[float, np.ndarray]] = {}
        self._last_meas_t: float = -1e9
        self._ticks = 0

        self.create_subscription(PoseStamped, "/target_pose", self._on_target, 10)
        self.create_subscription(String, "/drone_poses", self._on_drone_poses, 10)
        self.get_logger().info("L1 SLAMMOT node up: %d drones, %d signs, %d vehicles" % (
            len(self.drone_ids), len(self.scene.signs), len(self.scene.vehicles)))

    # ------------------------------------------------------------- truth in
    def _on_target(self, msg: PoseStamped) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        pos = np.array([p.x, p.y, p.z])
        vel = None
        if self._target is not None and t > self._target[0]:
            vel = (pos - self._target[1]) / (t - self._target[0])
        self._target = (t, pos, vel if vel is not None else (self._target[2] if self._target else None))

    def _on_drone_poses(self, msg: String) -> None:
        frame = DronePoseFrame.model_validate_json(msg.data)
        for pm in frame.poses:
            if pm.drone_id not in self.imus:
                self.get_logger().warning(f"pose for unknown drone {pm.drone_id} ignored")
                continue
            pos, q = np.asarray(pm.position), np.asarray(pm.orientation)
            prev = self._prev_pos.get(pm.drone_id)
            vel = (pos - prev[1]) / (pm.timestamp - prev[0]) if prev and pm.timestamp > prev[0] else np.zeros(3)
            self._prev_pos[pm.drone_id] = (pm.timestamp, pos)
            self._platforms[pm.drone_id] = PlatformTruth(pos, q)
            sample = self.imus[pm.drone_id].sample(pm.timestamp, vel, q)
            if sample is not None:
                self.pipeline.imu(pm.drone_id, sample)
        t = frame.timestamp
        if (t - self._last_meas_t) >= MEAS_DT and self._target is not None and self._target[2] is not None:
            self._last_meas_t = t
            self._fusion_tick(t)

    # ------------------------------------------------------------- fusion
    def _fusion_tick(self, t: float) -> None:
        assert self._target is not None and self._target[2] is not None
        _, pos, vel = self._target
        target = TargetTruth(np.concatenate([pos, vel, self.scene.target_extent]),
                             float(np.arctan2(vel[1], vel[0])) if np.linalg.norm(vel[:2]) > 1e-3 else 0.0,
                             self.scene.target_color)
        dets, _ = self.synth.observe(t, self._platforms, [target])
        res = self.pipeline.cycle(t, dets)
        self._ticks += 1
        self._publish(res.tracks, t)
        self.poses_pub.publish(String(data=res.poses.model_dump_json()))
        if self._ticks % 10 == 0:
            self._print_status(res.tracks, pos, t)

    def _print_status(self, confirmed, target_pos: np.ndarray, t: float) -> None:
        egos = " ".join(
            f"{did}:err={np.linalg.norm(ego.p - self._platforms[did].position):.2f}m "
            f"lm={len(ego.landmarks)}"
            for did, ego in self.pipeline.egos.items() if did in self._platforms)
        trk = "-"
        if confirmed:
            tr = confirmed[0]
            label, conf = (self.classifier.decide(tr.class_beliefs)
                           if tr.class_beliefs else ("-", 0.0))
            trk = (f"trk#{tr.track_id} age={tr.age} err={np.linalg.norm(tr.state.x[0:3] - target_pos):.2f}m "
                   f"class={label}({conf:.2f})")
        print(f"t={t:7.2f}s | {egos} | {trk} | tracks={len(self.pipeline.tracker.tracks)}", flush=True)

    def _publish(self, confirmed, t: float) -> None:
        msgs = []
        for tr in confirmed:
            e = tr.state.x
            label, conf = (self.classifier.decide(tr.class_beliefs)
                           if tr.class_beliefs else (None, 0.0))
            msgs.append(TrackMsg(
                track_id=tr.track_id, timestamp=t,
                position=[float(v) for v in e[0:3]], velocity=[float(v) for v in e[3:6]],
                extent=[float(v) for v in e[6:9]],
                position_sqrt_cov=[float(v) for v in sqrt_cov_block(tr.state).ravel()],
                class_label=label, class_confidence=float(conf),
                class_beliefs=encode_mass(tr.class_beliefs) if tr.class_beliefs else {},
                age=tr.age, source_sensor_ids=list(tr.source_sensor_ids)))
        self.tracks_pub.publish(String(data=TrackFrame(timestamp=t, tracks=msgs).model_dump_json()))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.0,
                        help="run for N wall seconds then exit (0 = until Ctrl+C)")
    args = parser.parse_args()

    rclpy.init()
    node = SlammotNode()
    deadline = time.time() + args.duration if args.duration > 0 else None
    try:
        while rclpy.ok() and (deadline is None or time.time() < deadline):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    n_conf = len([t for t in node.pipeline.tracker.tracks
                  if t.age >= node.pipeline.tracker.cfg.confirm_hits])
    print(f"\nFINAL: {node._ticks} cycles, {len(node.pipeline.tracker.tracks)} live tracks "
          f"({n_conf} confirmed); landmarks: " + ", ".join(
              f"{did}={len(ego.landmarks)}" for did, ego in node.pipeline.egos.items()), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
