from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray


@dataclass
class Detection:
    """A single sensor detection at one time step.

    Phase 1: camera bounding-box detections.
    Phase 2+: radar returns, lidar clusters — same interface, different obs spaces.
    """

    sensor_id: str
    timestamp: float
    measurement: NDArray[np.float64]  # shape depends on sensor type
    class_label: str | None = None
    class_confidence: float = 0.0


@dataclass
class TrackState:
    """Internal state of a single track maintained by the UKF."""

    x: NDArray[np.float64]  # state vector (state_dim,)
    P: NDArray[np.float64]  # covariance matrix (state_dim, state_dim)


@dataclass
class Track:
    """A confirmed track — the output of the tracker."""

    track_id: int
    state: TrackState
    class_beliefs: dict[str, float] = field(default_factory=dict)  # DS belief per class
    age: int = 0  # number of update cycles since initiation
    misses: int = 0  # consecutive update cycles with no associated detection
    source_sensor_ids: list[str] = field(default_factory=list)
