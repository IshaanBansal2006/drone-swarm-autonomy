"""Probabilistic data association (decision 011).

Ref: Bar-Shalom & Li, "Multitarget-Multisensor Tracking", Ch. 3.

Implementation note (documented in backlog): this is the PDA computation —
exact for a single target and for multi-target scenes whose gates do not
overlap. Full JPDA additionally enumerates joint association events so that
tracks competing for the same measurement share probability mass; that joint
layer lands with the multi-target milestone (the interfaces here don't change).
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from mini_lattice.edge.config import JPDAConfig
from mini_lattice.edge.types import Detection

# Gate probability: the chance a true detection falls inside the chi-squared
# 95% gate. Matches the CHI2_95 thresholds used per sensor.
P_GATE = 0.95


class JPDA:
    def __init__(self, config: JPDAConfig) -> None:
        self.cfg = config

    def gate(
        self,
        detections: list[Detection],
        z_pred: NDArray[np.float64],
        S: NDArray[np.float64],
        gate_threshold: float,
    ) -> list[int]:
        """Mahalanobis gating: indices of detections with d^T S^-1 d < threshold.

        The threshold is chi-squared in the MEASUREMENT dimension (per-sensor,
        from the sensor config), because the innovation is chi-squared with m
        DOF under the correct-association hypothesis.
        """
        S_inv = np.linalg.inv(S)
        gated: list[int] = []
        for i, det in enumerate(detections):
            d = det.measurement - z_pred
            if float(d @ S_inv @ d) < gate_threshold:
                gated.append(i)
        return gated

    def compute_association_probabilities(
        self,
        gated_detections: list[Detection],
        z_pred: NDArray[np.float64],
        S: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Association probabilities (parametric PDA).

        betas[0] = P(none of the gated detections is the target's), betas[j] =
        P(detection j is the target's). Each real hypothesis is weighted by
        P_D * N(z_j; z_pred, S) (detected AND looks like the prediction); the
        null hypothesis by lambda * (1 - P_D * P_G) (clutter density times
        "missed or fell outside the gate"). Bayes-normalized.
        """
        m = z_pred.shape[0]
        S_inv = np.linalg.inv(S)
        norm = 1.0 / np.sqrt((2.0 * np.pi) ** m * np.linalg.det(S))

        weights = [self.cfg.p_false_alarm * (1.0 - self.cfg.p_detection * P_GATE)]
        for det in gated_detections:
            d = det.measurement - z_pred
            likelihood = norm * float(np.exp(-0.5 * d @ S_inv @ d))
            weights.append(self.cfg.p_detection * likelihood)
        betas = np.asarray(weights)
        return betas / betas.sum()

    def compute_combined_innovation(
        self,
        gated_detections: list[Detection],
        betas: NDArray[np.float64],
        z_pred: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Probability-weighted innovation: sum_j betas[j] * (z_j - z_pred).

        The null hypothesis (betas[0]) contributes zero innovation, so the
        correction automatically shrinks when the target probably wasn't seen.
        """
        combined = np.zeros_like(z_pred)
        for j, det in enumerate(gated_detections):
            combined += betas[j + 1] * (det.measurement - z_pred)
        return combined
