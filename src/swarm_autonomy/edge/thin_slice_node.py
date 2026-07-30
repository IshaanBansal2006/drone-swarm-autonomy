"""Live L1 node: Isaac ground truth -> synthesized detections -> MultiTargetTracker.

Ported 2026-07-30 onto the REAL tracker (was a hand-rolled fusion loop): the
full decided L1 stack now runs against the sim — SR-UKF (014) + PDA association
(011) + two-point birth/lifecycle + Dempster-Shafer classification (012/015,
D-B7). The node's remaining harness role is measurement synthesis:

    Isaac /tf (truth pose) -> finite-difference velocity
        -> camera [u,v,w,h] + radar [range,az,el,doppler] via the REAL h() + noise
        -> Detection messages (with harness class labels for the cube)
        -> MultiTargetTracker.step()
        -> printed confirmed tracks + /tracks TrackFrame

/tracks carries a 040 `TrackFrame` as JSON in std_msgs/String. The String is
still an interim TRANSPORT (a custom .msg/IDL needs a colcon package), but the
CONTENT is now the validated schema — the hand-adapted short keys are gone.

Run (WSL, Isaac scene running):
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

from swarm_autonomy.edge.classification import DSClassifier
from swarm_autonomy.edge.config import TrackerConfig
from swarm_autonomy.edge.filters import sqrt_cov_block
from swarm_autonomy.edge.observation import CameraModel, h_camera, h_radar
from swarm_autonomy.edge.tracker import MultiTargetTracker
from swarm_autonomy.edge.types import Detection
from swarm_autonomy.schemas import TrackFrame, TrackMsg, encode_mass

TRUE_EXTENT = np.array([0.2, 0.2, 0.2])
CAM_OFFSET = np.array([-6.0, -2.0, 3.0])
RADAR_POS = np.zeros(3)
MEAS_DT = 0.1  # fusion cadence; must equal TrackerConfig.ukf.dt

R_CAMERA = np.diag([2.0**2, 2.0**2, 3.0**2, 3.0**2])
R_RADAR = np.diag([0.1**2, 0.01**2, 0.01**2, 0.1**2])

# Harness class labels for the cube: what a detector would attach. Camera is
# confident fine-grained; radar sees a broad ground-mover (per decision 012).
CAM_LABEL, CAM_CONF = "vehicle", 0.9
RAD_LABEL, RAD_CONF = "vehicle", 0.6


def chase_camera(target_pos: np.ndarray) -> CameraModel:
    """Follow-camera at a fixed offset, looking at the target (CV frame)."""
    cam_pos = target_pos + CAM_OFFSET
    z_c = (target_pos - cam_pos) / np.linalg.norm(target_pos - cam_pos)
    x_c = np.cross(z_c, [0.0, 0.0, 1.0])
    x_c /= np.linalg.norm(x_c)
    return CameraModel(fx=480.0, fy=480.0, cx=480.0, cy=300.0, width=960, height=600,
                       R_wc=np.stack([x_c, np.cross(z_c, x_c), z_c]), t_w=cam_pos)


class ThinSliceTracker(Node):
    def __init__(self) -> None:
        super().__init__("thin_slice_tracker")
        self.tracker = MultiTargetTracker(TrackerConfig())  # defaults: cam_front + radar_gs
        self.classifier = DSClassifier()
        self.rng = np.random.default_rng(42)
        self.tracks_pub = self.create_publisher(String, "/tracks", 10)

        self._last_pos: np.ndarray | None = None
        self._last_t: float | None = None
        self._v_true: np.ndarray | None = None
        self._last_meas_t: float = -1e9
        self._ticks = 0

        self.create_subscription(PoseStamped, "/target_pose", self._on_pose, 10)
        self.get_logger().info("subscribed to /target_pose; feeding MultiTargetTracker...")

    def _on_pose(self, msg: PoseStamped) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        pos = np.array([p.x, p.y, p.z])
        if self._last_t is not None and t > self._last_t:
            self._v_true = (pos - self._last_pos) / (t - self._last_t)
        self._last_pos, self._last_t = pos, t
        if self._v_true is not None and (t - self._last_meas_t) >= MEAS_DT:
            self._last_meas_t = t
            self._measurement_tick(pos, self._v_true, t)

    def _measurement_tick(self, pos: np.ndarray, vel: np.ndarray, t: float) -> None:
        x_true = np.concatenate([pos, vel, TRUE_EXTENT])
        cam = chase_camera(pos)
        dets = [
            Detection(sensor_id="radar_gs", timestamp=t,
                      measurement=h_radar(x_true, RADAR_POS)
                      + self.rng.multivariate_normal(np.zeros(4), R_RADAR),
                      class_label=RAD_LABEL, class_confidence=RAD_CONF),
            Detection(sensor_id="cam_front", timestamp=t,
                      measurement=h_camera(x_true, cam)
                      + self.rng.multivariate_normal(np.zeros(4), R_CAMERA),
                      class_label=CAM_LABEL, class_confidence=CAM_CONF),
        ]
        confirmed = self.tracker.step(dets, camera_models={"cam_front": cam})
        self._ticks += 1
        self._publish(confirmed, t)
        if self._ticks % 10 == 0 and confirmed:  # once per sim-second
            tr = confirmed[0]
            e = tr.state.x
            err = float(np.linalg.norm(e[0:3] - pos))
            label, conf = (self.classifier.decide(tr.class_beliefs)
                           if tr.class_beliefs else ("-", 0.0))
            print(
                f"t={t:7.2f}s | trk#{tr.track_id} age={tr.age:3d} | err={err:5.3f}m "
                f"| est v=({e[3]:5.2f},{e[4]:5.2f},{e[5]:5.2f}) "
                f"| est L=({e[6]:4.2f},{e[7]:4.2f},{e[8]:4.2f}) "
                f"| class={label}({conf:.2f}) | tracks={len(self.tracker.tracks)}",
                flush=True,
            )

    def _publish(self, confirmed, t: float) -> None:
        """Publish the complete confirmed picture as a 040 TrackFrame."""
        msgs = []
        for tr in confirmed:
            e = tr.state.x
            label, conf = (self.classifier.decide(tr.class_beliefs)
                           if tr.class_beliefs else (None, 0.0))
            msgs.append(TrackMsg(
                track_id=tr.track_id,
                timestamp=t,
                position=[float(v) for v in e[0:3]],
                velocity=[float(v) for v in e[3:6]],
                extent=[float(v) for v in e[6:9]],
                # row-major 3x3; L @ L.T is the marginal position covariance
                position_sqrt_cov=[float(v) for v in sqrt_cov_block(tr.state).ravel()],
                class_label=label,
                class_confidence=float(conf),
                # DS-native mass rides downstream (D-B7) so L3 can show Bel/Pl,
                # not just the flattened label
                class_beliefs=encode_mass(tr.class_beliefs) if tr.class_beliefs else {},
                age=tr.age,
                source_sensor_ids=list(tr.source_sensor_ids),
            ))
        frame = TrackFrame(timestamp=t, tracks=msgs)
        self.tracks_pub.publish(String(data=frame.model_dump_json()))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.0,
                        help="run for N wall seconds then exit (0 = until Ctrl+C)")
    args = parser.parse_args()

    rclpy.init()
    node = ThinSliceTracker()
    deadline = time.time() + args.duration if args.duration > 0 else None
    try:
        while rclpy.ok() and (deadline is None or time.time() < deadline):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    n_conf = len([t for t in node.tracker.tracks if t.age >= node.tracker.cfg.confirm_hits])
    print(f"\nFINAL: {node._ticks} cycles, {len(node.tracker.tracks)} live tracks "
          f"({n_conf} confirmed)", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
