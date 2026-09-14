"""Drone Swarm Autonomy thin-slice scene (Step 0 / Option B).

Lightweight ground-truth-pose scene: a ground plane, a light, and ONE cube
("target") that moves at constant velocity. Publishes:
  - /clock   (sim time)
  - /tf      (ground-truth pose of the target, frame "target" under "world")

The WSL tracker subscribes to /tf, corrupts it with noise, and runs it through
the UKF — proving sim -> ROS2 -> L1 end to end.

RUN ON WINDOWS (Isaac's bundled python, NOT the project venv):
  C:\\IsaacSim\\python.bat \\\\wsl.localhost\\Ubuntu-22.04\\home\\ishaan\\projects\\drone-swarm-autonomy\\sim\\scenes\\thin_slice.py

Grounded in the shipped examples for Isaac Sim 6.0.1:
  standalone_examples/api/isaacsim.ros2.bridge/clock.py
  standalone_examples/api/isaacsim.core.experimental.api/add_cubes.py
  standalone_examples/testing/isaacsim.ros2.bridge/test_publish_camera_data.py
Set HEADLESS=False below if you want to watch it in the GUI.
"""

from isaacsim import SimulationApp

HEADLESS = False
simulation_app = SimulationApp({"headless": HEADLESS})

# Omniverse imports must come AFTER SimulationApp is constructed.
import numpy as np  # noqa: E402
import usdrt  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.timeline  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane  # noqa: E402
from isaacsim.core.experimental.prims import GeomPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# Isaac's INTERNAL rclpy (compiled for its bundled ROS2 distro) — usable only
# after the bridge extension is enabled. Used to publish the target's pose
# directly from the motion loop: the OmniGraph ComputeTF->PublishTF chain
# proved flaky (branch silently dies after early ticks when a node errors and
# OmniGraph disables it — observed runs 4/5), while direct rclpy publishing is
# deterministic. /tf stays wired for reference; /target_pose is the truth feed.
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from std_msgs.msg import String  # noqa: E402

# --- Scene -----------------------------------------------------------------
stage_utils.create_new_stage()
stage_utils.set_stage_units(meters_per_unit=1.0)
GroundPlane("/World/GroundPlane", positions=[0.0, 0.0, 0.0])
_light = DistantLight("/World/DistantLight")
_light.set_intensities(300)

TARGET_PATH = "/World/target"
Cube(paths=TARGET_PATH, positions=[0.0, 0.0, 0.5], sizes=1.0, scales=[0.2, 0.2, 0.2])
target = GeomPrim(paths=TARGET_PATH)  # visual-only; we drive its pose kinematically

# Fleet prims (Step 1/2): kinematics run WSL-side in the L2 KinematicBackend
# (single source of truth); Isaac only RENDERS the poses arriving on
# /drone_poses. This inverts at the 008 fidelity-ladder upgrade, when real
# quadrotor dynamics move into the sim. Must match mission_node.FLEET ids.
DRONE_IDS = ["d0", "d1"]
drone_prims = {}
for i, did in enumerate(DRONE_IDS):
    path = f"/World/drone_{did}"
    Cube(paths=path, positions=[-5.0 + 10.0 * i, -5.0, 2.0], sizes=1.0,
         scales=[0.3, 0.3, 0.1])  # flat slab ~ quad silhouette
    drone_prims[did] = GeomPrim(paths=path)

# --- ROS2 action graph: /clock + /tf(target) -------------------------------
# (ROS2 publishers use the default context/domain, i.e. ROS_DOMAIN_ID — same as clock.py.)
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("ComputeTF", "isaacsim.core.nodes.IsaacComputeTransformTree"),
            ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("SimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("OnTick.outputs:tick", "ComputeTF.inputs:execIn"),
            ("ComputeTF.outputs:execOut", "PublishTF.inputs:execIn"),
            ("ComputeTF.outputs:parentFrames", "PublishTF.inputs:parentFrames"),
            ("ComputeTF.outputs:childFrames", "PublishTF.inputs:childFrames"),
            ("ComputeTF.outputs:translations", "PublishTF.inputs:translations"),
            ("ComputeTF.outputs:orientations", "PublishTF.inputs:orientations"),
            ("SimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ("Context.outputs:context", "PublishTF.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "/clock"),
            ("PublishTF.inputs:topicName", "/tf"),
            ("ComputeTF.inputs:targetPrims", [usdrt.Sdf.Path(TARGET_PATH)]),
        ],
    },
)

# Two updates between graph creation and play — matches the shipped clock.py
# example; without them the ROS2 publisher graph can race timeline start and
# come up dead (observed once: /clock published, /tf silent for a whole run).
simulation_app.update()
simulation_app.update()

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
app_utils.play()
simulation_app.update()

# --- Constant-velocity motion (deterministic ground truth) -----------------
# Motion is gated on the timeline so GUI Pause/Stop actually freezes the cube
# (an ungated python loop drives the pose regardless of playback state).
P0 = np.array([0.0, 0.0, 0.5])
VEL = np.array([0.5, 0.2, 0.0])  # m/s
DT = 1.0 / 60.0
t = 0.0
timeline = omni.timeline.get_timeline_interface()

rclpy.init()
ros_node = rclpy.create_node("thin_slice_scene")
pose_pub = ros_node.create_publisher(PoseStamped, "/target_pose", 10)

_latest_drone_poses = {}


def _on_drone_poses(msg):
    # 041 DronePoseFrame: {"timestamp": t, "poses": [{"drone_id", "position", "orientation"}]}
    for p in __import__("json").loads(msg.data)["poses"]:
        _latest_drone_poses[p["drone_id"]] = (p["position"], p["orientation"])


ros_node.create_subscription(String, "/drone_poses", _on_drone_poses, 10)

print("[thin_slice] publishing /clock, /tf and /target_pose. Ctrl+C to stop.")
while simulation_app.is_running():
    if timeline.is_playing():
        pos = P0 + VEL * t
        target.set_world_poses(positions=[pos.tolist()], orientations=[[1.0, 0.0, 0.0, 0.0]])
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int((t - int(t)) * 1e9)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos.tolist()
        msg.pose.orientation.w = 1.0
        pose_pub.publish(msg)
        # render the fleet at its latest commanded poses (L2 owns the dynamics)
        for did, prim in drone_prims.items():
            if did in _latest_drone_poses:
                pos_d, quat_d = _latest_drone_poses[did]
                prim.set_world_poses(positions=[pos_d], orientations=[quat_d])  # [w,x,y,z]
        t += DT
    rclpy.spin_once(ros_node, timeout_sec=0.0)  # ingest /drone_poses
    simulation_app.update()

ros_node.destroy_node()
rclpy.shutdown()

app_utils.stop()
simulation_app.close()
