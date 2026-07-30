"""Edge-layer configuration.

Rewritten 2026-07-29 per decisions 013 (9-D state, 3-D radar) and backlog D-B1
(sensors as an id-keyed pydantic discriminated union — matches decision 007's
mixed fixed/drone-mounted heterogeneous topology and `Detection.sensor_id`).
"""

from __future__ import annotations

from typing import Annotated, Literal, Union

from pydantic import BaseModel, Field, model_validator

# chi-squared 95% thresholds by measurement DOF (gate = d^2 < chi2)
CHI2_95 = {1: 3.84, 2: 5.99, 3: 7.81, 4: 9.49, 5: 11.07, 6: 12.59}


class UKFConfig(BaseModel):
    """UKF tuning parameters (state layout per decision 013)."""

    state_dim: int = 9  # [x, y, z, vx, vy, vz, Lx, Ly, Lz]

    # Sigma-point scaling. alpha=1, kappa=1 -> lambda=1 -> ALL weights positive
    # at n=9, so covariance reconstructions are PSD by construction. The
    # classic small-alpha default (1e-3) gives center weights ~ +/-1e6 at n=9
    # and drove P indefinite in live runs (see explanations/edge/ukf.md).
    alpha: float = 1.0
    beta: float = 2.0  # prior-distribution knowledge (2.0 = Gaussian-optimal)
    kappa: float = 1.0

    dt: float = 0.1  # prediction time step (s); fusion cadence must match

    # Covariance strategy (decision 014, bake-off 2026-07-29): "srukf" is the
    # DEFAULT — square-root filter carrying the Cholesky factor (cannot go
    # indefinite; best stress accuracy/consistency). "shortcut" (P - K S K^T)
    # and "joseph" (quadratic-in-K) remain selectable on the standard UKF for
    # comparison. Construct filters via edge.filters.make_filter(), which
    # dispatches on this field.
    covariance_form: Literal["srukf", "shortcut", "joseph"] = "srukf"

    # Process noise — diagonal of Q. Length must equal state_dim.
    # Extent Q = 1e-3 is NEES-tuned (D-B11 sweep, benchmarks/nees_diagnosis.py):
    # rigid bodies don't change size, but tiny Q_ext (1e-8) froze a biased
    # extent belief -> NEES_ext ~9,770 vs ideal 3 ("smug filter"). 1e-3 restores
    # honesty (NEES_ext ~25) and slightly improves accuracy. Here Q compensates
    # observability/model bias, not physical size drift; residual inconsistency
    # is structural (see backlog D-B11 resolution).
    process_noise: list[float] = [1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3]

    @model_validator(mode="after")
    def _q_matches_state(self) -> "UKFConfig":
        if len(self.process_noise) != self.state_dim:
            raise ValueError(
                f"process_noise has {len(self.process_noise)} entries; "
                f"state_dim={self.state_dim} — they must match."
            )
        return self


class CameraConfig(BaseModel):
    """A camera sensor: measures pixel bbox [u, v, w, h] (4-DOF)."""

    sensor_type: Literal["camera"] = "camera"
    # Measurement noise — diagonal of R (px^2 as std-devs listed): u, v, w, h.
    measurement_noise: list[float] = [2.0, 2.0, 3.0, 3.0]
    gate_threshold: float = CHI2_95[4]  # 4-DOF measurement
    # Pinhole intrinsics (calibration; extrinsics are per-step platform pose).
    fx: float = 480.0
    fy: float = 480.0
    cx: float = 480.0
    cy: float = 300.0
    width: int = 960
    height: int = 600


class RadarConfig(BaseModel):
    """A 3-D radar: measures [range, azimuth, elevation, doppler] (4-DOF).

    Elevation added per decision 013 amendment (2026-07-29) — radar alone
    recovers full 3-D position (sensor redundancy / graceful degradation).
    """

    sensor_type: Literal["radar"] = "radar"
    # std-devs: range (m), azimuth (rad), elevation (rad), doppler (m/s)
    measurement_noise: list[float] = [0.1, 0.01, 0.01, 0.1]
    gate_threshold: float = CHI2_95[4]  # 4-DOF measurement
    # Mounting position for FIXED sensors (world frame, m). Drone-mounted
    # radars get their pose per-step from the platform instead.
    position: list[float] | None = None


class LidarConfig(BaseModel):
    """Lidar centroid sensor: [x, y, z] (3-DOF). Phase 3 (decision 007) — stub."""

    sensor_type: Literal["lidar"] = "lidar"
    measurement_noise: list[float] = [0.05, 0.05, 0.05]
    gate_threshold: float = CHI2_95[3]  # 3-DOF measurement


# Discriminated union: pydantic dispatches on sensor_type, so each entry in the
# sensors dict validates against exactly its own schema (D-B1, option A).
SensorConfig = Annotated[
    Union[CameraConfig, RadarConfig, LidarConfig],
    Field(discriminator="sensor_type"),
]


class JPDAConfig(BaseModel):
    """JPDA association parameters.

    Gating thresholds live PER-SENSOR now (measurement DOF differs by sensor);
    see each sensor config's `gate_threshold`.
    """

    p_detection: float = 0.9  # probability sensor detects a target in gate
    p_false_alarm: float = 1e-5  # spatial density of false alarms


class TrackerConfig(BaseModel):
    """Top-level tracker config."""

    ukf: UKFConfig = UKFConfig()
    jpda: JPDAConfig = JPDAConfig()

    # Sensors keyed by sensor_id — the same id carried by Detection.sensor_id,
    # so "which model/noise produced this detection" is a single dict lookup.
    sensors: dict[str, SensorConfig] = {
        "cam_front": CameraConfig(),
        "radar_gs": RadarConfig(position=[0.0, 0.0, 0.0]),
    }

    # Track management (M-of-N confirmation per decision 011)
    confirm_hits: int = 3  # M hits ...
    confirm_window: int = 5  # ... in N scans to confirm
    max_misses: int = 5  # consecutive misses before deletion
