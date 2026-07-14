"""UKF tests — YOU write the assertions.

The test structure is here. Fill in assertions that verify your UKF
does what you expect. Run with: pytest tests/edge/test_ukf.py -v
"""

from __future__ import annotations

import numpy as np
import pytest

from mini_lattice.edge.config import UKFConfig
from mini_lattice.edge.types import TrackState
from mini_lattice.edge.ukf import UKF


@pytest.fixture
def ukf() -> UKF:
    return UKF(UKFConfig())


@pytest.fixture
def initial_state() -> TrackState:
    """A target at (10, 20, 0) moving at (1, 0, 0)."""
    return TrackState(
        x=np.array([10.0, 20.0, 0.0, 1.0, 0.0, 0.0]),
        P=np.eye(6) * 1.0,
    )


class TestWeights:
    def test_weights_sum_to_one(self, ukf: UKF) -> None:
        """Wm should sum to 1. Wc should sum to 1."""
        # YOUR ASSERTIONS HERE
        pass

    def test_weight_count(self, ukf: UKF) -> None:
        """Should have 2n+1 weights."""
        # YOUR ASSERTIONS HERE
        pass


class TestSigmaPoints:
    def test_sigma_point_count(self, ukf: UKF, initial_state: TrackState) -> None:
        """Should generate 2n+1 sigma points."""
        # YOUR ASSERTIONS HERE
        pass

    def test_sigma_point_mean_recovers_state(self, ukf: UKF, initial_state: TrackState) -> None:
        """Weighted mean of sigma points should equal the original state."""
        # YOUR ASSERTIONS HERE
        pass


class TestPredict:
    def test_constant_velocity_prediction(self, ukf: UKF, initial_state: TrackState) -> None:
        """After one predict step with dt=0.1, position should advance by velocity*dt."""
        # YOUR ASSERTIONS HERE
        pass

    def test_covariance_grows(self, ukf: UKF, initial_state: TrackState) -> None:
        """Predicted covariance should be larger than initial (process noise added)."""
        # YOUR ASSERTIONS HERE
        pass


class TestUpdate:
    def test_update_pulls_toward_measurement(self, ukf: UKF, initial_state: TrackState) -> None:
        """After update, state should move toward the measurement."""
        # YOUR ASSERTIONS HERE
        pass

    def test_covariance_shrinks_after_update(self, ukf: UKF, initial_state: TrackState) -> None:
        """Updated covariance should be smaller than predicted covariance."""
        # YOUR ASSERTIONS HERE
        pass
