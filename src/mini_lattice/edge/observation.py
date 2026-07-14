"""Observation models — one per sensor type.

Each function maps a state vector to the measurement space of that sensor.
These are passed to UKF.update() as the `h` argument.

YOU implement each observation model.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


def h_camera(x: NDArray[np.float64]) -> NDArray[np.float64]:
    """Camera observation model: state -> pixel-space bounding box.

    Args:
        x: state vector (n,). If state is [x,y,z,vx,vy,vz], use position components.

    Returns:
        z_pred: predicted measurement in camera space.

    Design decision (yours to make):
        - If you're tracking in 3D world coordinates, this needs a projection
          (camera intrinsics + extrinsics) to map [x,y,z] -> [u,v] pixel center.
        - If you're tracking in 2D image coordinates for Phase 1, this may be
          a simple extraction of position components.
        - Bbox width/height: model as part of state, or treat as independent?

    Phase 1: pick the simpler option to get the pipeline working.
    Phase 2: when radar arrives, the state MUST be 3D so both sensors share it.
    """
    raise NotImplementedError("YOUR IMPLEMENTATION — camera observation model")


def h_radar(x: NDArray[np.float64]) -> NDArray[np.float64]:
    """Radar observation model: state -> [range, bearing, doppler].

    Phase 2 — leave this for later.
    """
    raise NotImplementedError("Phase 2")


def h_lidar(x: NDArray[np.float64]) -> NDArray[np.float64]:
    """Lidar observation model: state -> [x, y, z] centroid.

    Phase 3 — leave this for later.
    """
    raise NotImplementedError("Phase 3")
