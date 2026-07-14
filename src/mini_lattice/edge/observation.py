"""Observation models — one h(x) per sensor type.

Each maps a state vector to the measurement space of that sensor.
Passed to UKF.update() as the `h` argument.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


def h_camera(x: NDArray[np.float64]) -> NDArray[np.float64]:
    """Camera observation model: state -> pixel-space measurement."""
    raise NotImplementedError


def h_radar(x: NDArray[np.float64]) -> NDArray[np.float64]:
    """Radar observation model: state -> [range, bearing, doppler]. Phase 2."""
    raise NotImplementedError


def h_lidar(x: NDArray[np.float64]) -> NDArray[np.float64]:
    """Lidar observation model: state -> [x, y, z] centroid. Phase 3."""
    raise NotImplementedError
