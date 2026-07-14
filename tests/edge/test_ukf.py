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
    return TrackState(
        x=np.array([10.0, 20.0, 0.0, 1.0, 0.0, 0.0]),
        P=np.eye(6) * 1.0,
    )


class TestWeights:
    def test_weights_sum_to_one(self, ukf: UKF) -> None:
        pass

    def test_weight_count(self, ukf: UKF) -> None:
        pass


class TestSigmaPoints:
    def test_sigma_point_count(self, ukf: UKF, initial_state: TrackState) -> None:
        pass

    def test_sigma_point_mean_recovers_state(self, ukf: UKF, initial_state: TrackState) -> None:
        pass


class TestPredict:
    def test_constant_velocity_prediction(self, ukf: UKF, initial_state: TrackState) -> None:
        pass

    def test_covariance_grows(self, ukf: UKF, initial_state: TrackState) -> None:
        pass


class TestUpdate:
    def test_update_pulls_toward_measurement(self, ukf: UKF, initial_state: TrackState) -> None:
        pass

    def test_covariance_shrinks_after_update(self, ukf: UKF, initial_state: TrackState) -> None:
        pass
