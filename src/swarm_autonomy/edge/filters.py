"""Filter factory — constructs the estimator selected by config (decision 014).

`UKFConfig.covariance_form` picks the strategy; consumers call `make_filter`
and `initial_state` instead of naming a filter class, so swapping strategies
(or re-running the D-B2 bake-off) never touches call sites. UKF and
SquareRootUKF share the same predict/update signatures; only the state carrier
differs (P vs its Cholesky factor), which `initial_state` hides.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from swarm_autonomy.edge.config import UKFConfig
from swarm_autonomy.edge.srukf import SquareRootUKF, SRTrackState
from swarm_autonomy.edge.types import TrackState
from swarm_autonomy.edge.ukf import UKF


def make_filter(config: UKFConfig) -> UKF | SquareRootUKF:
    """SR-UKF for "srukf" (the default), standard UKF for "shortcut"/"joseph"."""
    if config.covariance_form == "srukf":
        return SquareRootUKF(config)
    return UKF(config)


def initial_state(
    filt: UKF | SquareRootUKF,
    x0: NDArray[np.float64],
    P0: NDArray[np.float64],
) -> TrackState | SRTrackState:
    """Build the filter-appropriate state from a mean and covariance."""
    if isinstance(filt, SquareRootUKF):
        return SRTrackState(x=x0, S=np.linalg.cholesky(P0))
    return TrackState(x=x0, P=P0)
