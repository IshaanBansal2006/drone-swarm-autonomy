"""L3 COP — Rerun bridge (decision 030): the operator's live 3-D picture.

Subscribes the system's own feeds (ROS 2 stays the inter-layer truth; Rerun is
a TAP, not a hop in the pipeline):
    /tracks        -> 3-D boxes sized by fused extent, labeled class(conf)
    /drone_poses   -> fleet points + heading arrows (truth, from L2)
    /drone_pose_estimates -> L1's estimate: points + position-uncertainty ellipsoids
    /mission_status-> text panel

Timeline is sim time (`payload["t"]`), so the Rerun scrubber replays the
mission. Run with --save out.rrd for a headless recording (also how tests and
CI can exercise this without a viewer); default spawns the viewer (WSLg).
"""

from __future__ import annotations

import argparse
import json

import numpy as np
import rclpy
import rerun as rr
from rclpy.node import Node
from std_msgs.msg import String

from swarm_autonomy.edge import rotation
from swarm_autonomy.schemas import DronePoseFrame, TrackFrame


def _set_time(t: float) -> None:
    """Rerun >=0.23 renamed set_time_seconds -> set_time(duration=...)."""
    rr.set_time("sim_time", duration=t)


class RerunBridge(Node):
    def __init__(self) -> None:
        super().__init__("cop_rerun_bridge")
        self.create_subscription(String, "/tracks", self._on_tracks, 10)
        self.create_subscription(String, "/drone_poses", self._on_drones, 10)
        self.create_subscription(String, "/drone_pose_estimates", self._on_estimates, 10)
        self.create_subscription(String, "/mission_status", self._on_status, 10)
        self._t = 0.0
        self.get_logger().info("COP bridge up — logging to Rerun")

    def _on_tracks(self, msg: String) -> None:
        frame = TrackFrame.model_validate_json(msg.data)
        # The envelope timestamp advances the scrubber even on an EMPTY frame —
        # "L1 is alive and confirms nothing" is a state the operator must see.
        self._t = frame.timestamp
        _set_time(self._t)
        for tr in frame.tracks:
            path = f"world/tracks/{tr.track_id}"
            rr.log(path, rr.Boxes3D(
                centers=[tr.position],
                half_sizes=[[max(e, 0.05) / 2 for e in tr.extent]],
                labels=[f"#{tr.track_id} {tr.class_label or '?'}"
                        f"({tr.class_confidence:.2f})"],
                colors=[[255, 80, 80]]))
            rr.log(path + "/vel", rr.Arrows3D(
                origins=[tr.position], vectors=[tr.velocity],
                colors=[[255, 160, 80]]))

    def _on_drones(self, msg: String) -> None:
        _set_time(self._t)
        frame = DronePoseFrame.model_validate_json(msg.data)
        positions = [p.position for p in frame.poses]
        rr.log("world/drones", rr.Points3D(
            positions=positions, radii=0.25,
            labels=[p.drone_id for p in frame.poses], colors=[[80, 160, 255]]))
        rr.log("world/drones/heading", rr.Arrows3D(
            origins=positions,
            vectors=[rotation.rotate(np.asarray(p.orientation), np.array([1.5, 0.0, 0.0])).tolist()
                     for p in frame.poses],
            colors=[[80, 160, 255]]))

    def _on_estimates(self, msg: String) -> None:
        """L1's belief about the fleet. The ellipsoid is 2-sigma per axis from
        the MARGINAL position std-devs (diagonal of L L^T for the leading 3x3
        block of the pose factor) — axis-aligned, so it under-draws correlated
        error; enough to see uncertainty grow between sign sightings."""
        _set_time(self._t)
        frame = DronePoseFrame.model_validate_json(msg.data)
        centers, half_sizes = [], []
        for p in frame.poses:
            if p.pose_sqrt_cov is None:
                continue
            L = np.asarray(p.pose_sqrt_cov).reshape(6, 6)[:3, :3]
            centers.append(p.position)
            half_sizes.append((2.0 * np.sqrt(np.diag(L @ L.T))).tolist())
        rr.log("world/drone_estimates", rr.Points3D(
            positions=[p.position for p in frame.poses], radii=0.15,
            labels=[f"{p.drone_id}^" for p in frame.poses], colors=[[255, 220, 80]]))
        if centers:
            rr.log("world/drone_estimates/cov", rr.Ellipsoids3D(
                centers=centers, half_sizes=half_sizes, colors=[[255, 220, 80, 60]]))

    def _on_status(self, msg: String) -> None:
        _set_time(self._t)
        rr.log("mission/status", rr.TextDocument(
            json.dumps(json.loads(msg.data), indent=2)))


def main() -> None:  # pragma: no cover — thin ROS wrapper
    parser = argparse.ArgumentParser()
    parser.add_argument("--save", help="write a .rrd recording instead of serving")
    args = parser.parse_args()
    rr.init("drone-swarm-autonomy-cop")
    if args.save:
        rr.save(args.save)
    else:
        # WSL: the native viewer can't spawn (no local window server for it),
        # so serve gRPC + the WEB viewer — with mirrored networking the Windows
        # browser reaches it straight on localhost.
        uri = rr.serve_grpc()
        rr.serve_web_viewer(open_browser=False, connect_to=uri)
        print(f"\n>>> COP viewer: open http://localhost:9090/?url={uri} "
              f"in your Windows browser <<<\n", flush=True)
    rclpy.init()
    node = RerunBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
