"""Unscented Kalman Filter.

Sensor-agnostic — observation model h(x) is passed in at update time.
Ref: Wan & Van der Merwe (2000), "The Unscented Kalman Filter for Nonlinear Estimation".
"""

from __future__ import annotations

from typing import Callable

import numpy as np
from numpy.typing import NDArray

from mini_lattice.edge.config import UKFConfig
from mini_lattice.edge.types import TrackState


class UKF:
    def __init__(self, config: UKFConfig) -> None:
        self.cfg = config
        self.n = config.state_dim
        self.Q = np.diag(config.process_noise)
        self.Wm, self.Wc = self._compute_weights()

    def _compute_weights(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Sigma point weights Wm (mean) and Wc (covariance), shape (2n+1,) each.

        lambda = alpha^2 * (n + kappa) - n
        Wm[0] = lambda / (n + lambda)
        Wc[0] = Wm[0] + (1 - alpha^2 + beta)
        Wm[i] = Wc[i] = 1 / (2(n + lambda))   for i = 1..2n
        """
        raise NotImplementedError

    def generate_sigma_points(self, state: TrackState) -> NDArray[np.float64]:
        """Generate 2n+1 sigma points from state (x, P).

        Returns shape (2n+1, n). Uses Cholesky of (n + lambda) * P.
        """
        raise NotImplementedError

    def predict(self, state: TrackState) -> TrackState:
        """Predict state forward by dt. Unscented transform through f(x), add Q."""
        raise NotImplementedError

    def f(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        """Motion model. Propagate state vector (n,) forward by dt."""
        raise NotImplementedError

    def update(
        self,
        state: TrackState,
        measurement: NDArray[np.float64],
        h: Callable[[NDArray[np.float64]], NDArray[np.float64]],
        R: NDArray[np.float64],
    ) -> tuple[TrackState, NDArray[np.float64], NDArray[np.float64]]:
        """Incorporate measurement via unscented transform through h(x).

        Args:
            state: predicted state
            measurement: sensor reading, shape (m,)
            h: observation model, state (n,) -> measurement (m,)
            R: measurement noise covariance (m, m)

        Returns:
            (updated_state, innovation, innovation_covariance S)
        """
        raise NotImplementedError
