"""Unscented Kalman Filter.

YOU implement every method marked with NotImplementedError.
The UKF is sensor-agnostic — the observation model h(x) is passed in at update time.
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

        # --- Sigma point weights ---
        # Compute Wm (mean weights) and Wc (covariance weights) from alpha, beta, kappa.
        # Number of sigma points = 2n + 1.
        # See: Wan & Van der Merwe (2000), "The Unscented Kalman Filter for
        #      Nonlinear Estimation", Section 3.
        self.Wm, self.Wc = self._compute_weights()

    def _compute_weights(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Compute sigma point weights Wm and Wc.

        Returns:
            Wm: mean weights, shape (2n+1,)
            Wc: covariance weights, shape (2n+1,)

        Reference equations:
            lambda = alpha^2 * (n + kappa) - n
            Wm[0] = lambda / (n + lambda)
            Wc[0] = lambda / (n + lambda) + (1 - alpha^2 + beta)
            Wm[i] = Wc[i] = 1 / (2(n + lambda))   for i = 1..2n
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — sigma point weights")

    def generate_sigma_points(self, state: TrackState) -> NDArray[np.float64]:
        """Generate 2n+1 sigma points from the current state estimate.

        Args:
            state: current track state (x, P)

        Returns:
            sigma_points: shape (2n+1, n)

        Steps:
            1. Compute the matrix square root of (n + lambda) * P.
               Use np.linalg.cholesky.
            2. Sigma point 0 = x (the mean).
            3. Sigma points 1..n = x + row i of the square root.
            4. Sigma points n+1..2n = x - row i of the square root.
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — sigma point generation")

    def predict(self, state: TrackState) -> TrackState:
        """Predict state forward by dt using the motion model.

        Args:
            state: current state

        Returns:
            predicted state (new x, new P)

        Steps:
            1. Generate sigma points from current state.
            2. Pass each sigma point through the motion model f(x).
            3. Compute predicted mean: x_pred = sum(Wm[i] * sigma_i).
            4. Compute predicted covariance: P_pred = sum(Wc[i] * (sigma_i - x_pred)(sigma_i - x_pred)^T) + Q.
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — predict step")

    def f(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        """Motion model: propagate a single state vector forward by dt.

        Args:
            x: state vector (n,)

        Returns:
            x_next: predicted state vector (n,)

        For a constant-velocity model with state [x,y,z,vx,vy,vz]:
            position += velocity * dt
            velocity stays the same
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — motion model")

    def update(
        self,
        state: TrackState,
        measurement: NDArray[np.float64],
        h: Callable[[NDArray[np.float64]], NDArray[np.float64]],
        R: NDArray[np.float64],
    ) -> tuple[TrackState, NDArray[np.float64], NDArray[np.float64]]:
        """Update state with a measurement using the unscented transform.

        Args:
            state: predicted state (from predict step)
            measurement: actual sensor reading, shape (m,)
            h: observation model function, maps state (n,) -> predicted measurement (m,)
            R: measurement noise covariance, shape (m, m)

        Returns:
            updated_state: posterior state after incorporating measurement
            innovation: z - z_pred, shape (m,)
            S: innovation covariance, shape (m, m)

        Steps:
            1. Generate sigma points from the predicted state.
            2. Pass each sigma point through h(x) to get predicted measurement sigma points.
            3. Compute predicted measurement mean: z_pred = sum(Wm[i] * z_sigma_i).
            4. Compute innovation covariance: S = sum(Wc[i] * (z_sigma_i - z_pred)(...)^T) + R.
            5. Compute cross-covariance: Pxz = sum(Wc[i] * (x_sigma_i - x_pred)(z_sigma_i - z_pred)^T).
            6. Kalman gain: K = Pxz @ S^-1.
            7. Updated mean: x_new = x_pred + K @ (z - z_pred).
            8. Updated covariance: P_new = P_pred - K @ S @ K^T.
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — update step")
