"""Observation models — one h(x) per sensor type.

Each maps a target state to the measurement space of that sensor. Passed to
`UKF.update()` as the `h` argument — sensors that need a pose (camera, radar)
are bound to the current sensor pose via a per-step closure built by the tracker.

State layout (option A, 9-D — see docs/decisions/013):
    x = [px, py, pz, vx, vy, vz, Lx, Ly, Lz]
      position (m) | velocity (m/s) | physical 3-D extent (m)

Orientation is derived from the velocity heading (`_object_yaw`). The option-C
upgrade (yaw as an explicit state) is a one-function swap in `_object_yaw`
plus a state_dim bump — see the decision doc.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

_SPEED_EPS = 1e-3  # below this planar speed, velocity heading is undefined
_Z_EPS = 1e-6  # guard against division by ~0 depth


@dataclass
class CameraModel:
    """Pinhole intrinsics + the camera's current world pose (extrinsics).

    Set fx, fy, cx, cy, width, height from the Isaac camera. `R_wc` / `t_w` are
    the extrinsics at this instant; the drone moves, so the tracker rebuilds and
    binds them every cycle.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    R_wc: NDArray[np.float64]  # (3, 3) world -> camera rotation
    t_w: NDArray[np.float64]  # (3,) camera position in world frame


def _object_yaw(x: NDArray[np.float64]) -> float:
    """Yaw seam. Option A: heading from velocity. Option C: `return float(x[9])`.

    Planar speed below `_SPEED_EPS` (hover/static) leaves heading undefined; we
    return 0.0. That is the low-speed degradation of the velocity-as-orientation
    proxy — the case option C exists to fix.
    """
    vx, vy = float(x[3]), float(x[4])
    if np.hypot(vx, vy) < _SPEED_EPS:
        return 0.0
    return float(np.arctan2(vy, vx))


def _box_corners(x: NDArray[np.float64]) -> NDArray[np.float64]:
    """The 8 world-frame corners of the upright, yaw-rotated 3-D box, shape (8, 3).

    Upright assumption: rotation is only about world-z by yaw (roll/pitch ~ 0).
    """
    center = x[0:3]
    half = x[6:9] / 2.0
    yaw = _object_yaw(x)
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    signs = np.array(
        [[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]
    )
    local = signs * half  # (8, 3) corner offsets in the object frame
    return (Rz @ local.T).T + center  # rotate, then translate to world


def _corners_camera(x: NDArray[np.float64], cam: CameraModel) -> NDArray[np.float64]:
    """The 8 box corners in the camera frame (x right, y down, z forward), shape (8, 3)."""
    return (cam.R_wc @ (_box_corners(x) - cam.t_w).T).T


def _project(
    corners_c: NDArray[np.float64], z: NDArray[np.float64], cam: CameraModel
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Pinhole perspective divide: camera-frame points -> pixel coordinates (u, v)."""
    return cam.fx * corners_c[:, 0] / z + cam.cx, cam.fy * corners_c[:, 1] / z + cam.cy


def h_camera(x: NDArray[np.float64], cam: CameraModel) -> NDArray[np.float64]:
    """Camera observation model: state -> pixel bbox [u_center, v_center, w, h].

    Projects the 8 box corners through the pinhole model and returns the
    axis-aligned image bounding box (center + pixel width/height). Only valid
    where `in_view` holds.
    """
    corners_c = _corners_camera(x, cam)
    z = corners_c[:, 2]
    z = np.where(np.abs(z) < _Z_EPS, _Z_EPS, z)  # numerical guard only; see in_view

    u, v = _project(corners_c, z, cam)

    u_min, u_max = float(u.min()), float(u.max())
    v_min, v_max = float(v.min()), float(v.max())
    return np.array(
        [0.5 * (u_min + u_max), 0.5 * (v_min + v_max), u_max - u_min, v_max - v_min]
    )


def in_view(x: NDArray[np.float64], cam: CameraModel) -> bool:
    """Whether `h_camera` models what this camera would actually report for `x`.

    Requires every corner in front of the camera and the whole projected box
    inside the image (decision 017). A corner behind the camera flips sign under
    the perspective divide, and an edge-truncated detection is narrower than the
    untruncated box `h_camera` predicts — which the filter would read as the
    target being farther away. Either would hand the filter a measurement its
    model cannot explain, so the harness emits no detection instead.
    """
    corners_c = _corners_camera(x, cam)
    z = corners_c[:, 2]
    if bool(np.any(z <= _Z_EPS)):
        return False
    u, v = _project(corners_c, z, cam)
    return bool(
        u.min() >= 0.0 and u.max() <= cam.width and v.min() >= 0.0 and v.max() <= cam.height
    )


def h_radar(
    x: NDArray[np.float64],
    sensor_pos: NDArray[np.float64],
    sensor_vel: NDArray[np.float64] | None = None,
) -> NDArray[np.float64]:
    """Radar observation model: state -> [range, azimuth, elevation, doppler].

    3-D radar (decision 013 amendment 2026-07-29, matching decision 007's
    RadarReturn schema): slant range, azimuth bearing, elevation angle, and
    radial (Doppler) velocity. Radar alone now recovers full 3-D position —
    redundant with the camera by design (graceful degradation), and elevation
    directly observes the vertical channel (the weakest axis under 2-D radar).

    `sensor_vel` compensates for platform ego-motion in the Doppler term; defaults
    to a stationary sensor.
    """
    if sensor_vel is None:
        sensor_vel = np.zeros(3)

    rel = x[0:3] - sensor_pos
    rng = float(np.linalg.norm(rel))
    azimuth = float(np.arctan2(rel[1], rel[0]))
    horiz = float(np.hypot(rel[0], rel[1]))
    elevation = float(np.arctan2(rel[2], horiz))  # angle above the sensor's horizon
    if rng < _Z_EPS:
        doppler = 0.0
    else:
        r_hat = rel / rng
        doppler = float(np.dot(x[3:6] - sensor_vel, r_hat))  # radial closing speed
    return np.array([rng, azimuth, elevation, doppler])


def h_lidar(x: NDArray[np.float64]) -> NDArray[np.float64]:
    """Lidar observation model: state -> [x, y, z] centroid. Phase 3 (stub)."""
    raise NotImplementedError
