from __future__ import annotations

from pydantic import BaseModel


class UKFConfig(BaseModel):
    """UKF tuning parameters."""

    state_dim: int = 6  # e.g. [x, y, z, vx, vy, vz] — your call on what the state represents
    alpha: float = 1e-3  # sigma point spread
    beta: float = 2.0  # prior knowledge (2.0 = Gaussian)
    kappa: float = 0.0  # secondary scaling

    dt: float = 0.1  # prediction time step (seconds)

    # Process noise — how much you trust the motion model.
    # Diagonal of Q. Length must equal state_dim. Tune these.
    process_noise: list[float] = [1.0, 1.0, 1.0, 0.5, 0.5, 0.5]


class SensorConfig(BaseModel):
    """Per-sensor-type measurement noise."""

    sensor_type: str = "camera"  # "camera", "radar", "lidar"

    # Measurement noise — diagonal of R. Length = measurement_dim for this sensor.
    # Camera (Phase 1): measurement is [u, v, w, h] (pixel bbox) → 4 values.
    measurement_noise: list[float] = [5.0, 5.0, 10.0, 10.0]


class JPDAConfig(BaseModel):
    """JPDA gating and association parameters."""

    gate_threshold: float = 9.21  # chi-squared gate (95% with 2 DOF)
    p_detection: float = 0.9  # probability sensor detects a target in gate
    p_false_alarm: float = 1e-5  # spatial density of false alarms


class TrackerConfig(BaseModel):
    """Top-level tracker config."""

    ukf: UKFConfig = UKFConfig()
    sensor: SensorConfig = SensorConfig()
    jpda: JPDAConfig = JPDAConfig()

    # Track management
    confirm_hits: int = 3  # M hits in N scans to confirm
    confirm_window: int = 5
    max_misses: int = 5  # consecutive misses before deletion
