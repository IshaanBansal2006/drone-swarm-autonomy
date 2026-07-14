"""Joint Probabilistic Data Association.

Ref: Bar-Shalom & Li, "Multitarget-Multisensor Tracking", Ch. 3.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from mini_lattice.edge.config import JPDAConfig
from mini_lattice.edge.types import Detection, TrackState


class JPDA:
    def __init__(self, config: JPDAConfig) -> None:
        self.cfg = config

    def gate(
        self,
        track: TrackState,
        detections: list[Detection],
        z_pred: NDArray[np.float64],
        S: NDArray[np.float64],
    ) -> list[int]:
        """Mahalanobis gating. Returns indices of detections inside the gate."""
        raise NotImplementedError

    def compute_association_probabilities(
        self,
        track: TrackState,
        gated_detections: list[Detection],
        z_pred: NDArray[np.float64],
        S: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Association probabilities for each gated detection.

        Returns betas, shape (len(gated_detections) + 1,).
        betas[0] = P(no detection is correct), betas[j] = P(detection j is correct).
        """
        raise NotImplementedError

    def compute_combined_innovation(
        self,
        gated_detections: list[Detection],
        betas: NDArray[np.float64],
        z_pred: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Probability-weighted combined innovation, shape (m,)."""
        raise NotImplementedError
