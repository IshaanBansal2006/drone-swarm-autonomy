"""Joint Probabilistic Data Association.

YOU implement every method marked with NotImplementedError.
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
        z_preds: NDArray[np.float64],
        S: NDArray[np.float64],
    ) -> list[int]:
        """Return indices of detections that fall within the validation gate.

        Args:
            track: predicted track state
            detections: list of detections this scan
            z_preds: predicted measurement for this track, shape (m,)
            S: innovation covariance, shape (m, m)

        Returns:
            list of indices into `detections` that pass the gate

        Steps:
            1. For each detection, compute the Mahalanobis distance:
               d^2 = (z - z_pred)^T @ S^-1 @ (z - z_pred)
            2. Keep if d^2 < gate_threshold (chi-squared threshold).
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — Mahalanobis gating")

    def compute_association_probabilities(
        self,
        track: TrackState,
        gated_detections: list[Detection],
        z_pred: NDArray[np.float64],
        S: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Compute the probability that each gated detection belongs to this track.

        Args:
            track: predicted track state
            gated_detections: detections that passed the gate
            z_pred: predicted measurement, shape (m,)
            S: innovation covariance, shape (m, m)

        Returns:
            betas: shape (len(gated_detections) + 1,)
                   betas[0] = probability no detection is correct (missed detection)
                   betas[j] = probability detection j is correct, for j=1..n

        Steps:
            1. For each gated detection j, compute the likelihood:
               L_j = N(z_j; z_pred, S)  (multivariate Gaussian PDF)
            2. beta_0 = (1 - p_detection) * p_false_alarm_volume_term
            3. beta_j = p_detection * L_j / normalizer
            4. Normalize so sum(betas) = 1.

        Reference: Bar-Shalom & Li, "Multitarget-Multisensor Tracking", Ch. 3.
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — association probabilities")

    def compute_combined_innovation(
        self,
        gated_detections: list[Detection],
        betas: NDArray[np.float64],
        z_pred: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Compute the probability-weighted combined innovation for the UKF update.

        Args:
            gated_detections: detections that passed the gate
            betas: association probabilities from compute_association_probabilities
            z_pred: predicted measurement, shape (m,)

        Returns:
            combined_innovation: shape (m,)

        This is the weighted sum: sum(beta_j * (z_j - z_pred)) for j=1..n.
        beta_0 contributes a zero innovation (no detection = no correction).
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — combined innovation")
