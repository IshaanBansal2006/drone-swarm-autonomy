"""Drone Swarm Autonomy road scene (Step 0 thin slice, extended for SLAMMOT — decisions 017/018).

Ground-truth-pose scene: a ground plane, a light, the road-patrol layout from
`swarm_autonomy.scene` (prior-mapped road signs and parked vehicles as boxes),
and ONE vehicle-sized box ("target") driving down the road at constant
velocity. Publishes:
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

import os
import sys

from isaacsim import SimulationApp

HEADLESS = False
simulation_app = SimulationApp({"headless": HEADLESS})

# The shared scene description (pure dataclasses + numpy; no package install
# needed under Isaac's bundled Python). Path is relative to this file so the
# C:\ copy and the WSL original both resolve it.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))
from swarm_autonomy.scene import demo_scene  # noqa: E402

SCENE = demo_scene()

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

def _yaw_quat(yaw):
    return [float(np.cos(0.5 * yaw)), 0.0, 0.0, float(np.sin(0.5 * yaw))]


TARGET_PATH = "/World/target"
Cube(paths=TARGET_PATH, positions=[list(SCENE.target_start)], sizes=1.0,
     scales=[list(SCENE.target_extent)])
target = GeomPrim(paths=TARGET_PATH)  # visual-only; we drive its pose kinematically

# Landmarks (decision 018). Colours are NOT rendered here: the measurement
# synthesiser reads colour from the scene description, not from pixels, so
# a material is cosmetic for this harness. Sizes and poses ARE what the
# synthesiser projects, so they must match `scene.py` exactly — hence the import.
for sign in SCENE.signs:
    Cube(paths=f"/World/sign_{sign.sign_id}", positions=[list(sign.position)], sizes=1.0,
         scales=[sign.extent.tolist()], orientations=[_yaw_quat(sign.yaw)])
    Cube(paths=f"/World/signpost_{sign.sign_id}", sizes=1.0, scales=[0.05, 0.05, sign.position[2]],
         positions=[[sign.position[0], sign.position[1], sign.position[2] / 2.0]])
for veh in SCENE.vehicles:
    Cube(paths=f"/World/vehicle_{veh.vehicle_id}", positions=[list(veh.position)], sizes=1.0,
         scales=[veh.extent.tolist()], orientations=[_yaw_quat(veh.yaw)])

# Fleet prims (Step 1/2): kinematics run WSL-side in the L2 KinematicBackend
# (single source of truth); Isaac only RENDERS the poses arriving on
# /drone_poses. This inverts at the 008 fidelity-ladder upgrade, when real
# quadrotor dynamics move into the sim. Must match mission_node.FLEET ids.
DRONE_IDS = sorted(SCENE.fleet)
drone_prims = {}
for did in DRONE_IDS:
    path = f"/World/drone_{did}"
    Cube(paths=path, positions=[list(SCENE.fleet[did].position)], sizes=1.0,
         scales=[0.3, 0.3, 0.1],  # flat slab ~ quad silhouette
         orientations=[_yaw_quat(SCENE.fleet[did].yaw)])
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
P0 = np.array(SCENE.target_start)
VEL = np.array(SCENE.target_velocity)  # m/s, down the road
TARGET_QUAT = _yaw_quat(float(np.arctan2(VEL[1], VEL[0])))
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
        target.set_world_poses(positions=[pos.tolist()], orientations=[TARGET_QUAT])
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
