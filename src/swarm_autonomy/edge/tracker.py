"""Multi-sensor multi-target tracker — wires filter (014) + PDA (011) + lifecycle.

Rewritten 2026-07-29: consumes the id-keyed sensor dict (D-B1), gates with the
REAL innovation covariance from the filter's unscented measurement prediction
(closing old gap #1), and initiates tracks by two-point differencing of radar
detections (closing old gap #2 — the tracker can now bootstrap from empty).

Per-cycle flow (call step() once per UKFConfig.dt of sim time):
    predict all tracks
    for each sensor with detections:  gate -> PDA betas -> combined innovation
                                      -> pseudo-measurement filter update
    birth: unassociated radar detections paired across consecutive cycles
    lifecycle: hits/misses -> confirm (age >= confirm_hits) / delete (max_misses)

Documented simplifications (see docs/backlog.md):
  - PDA, not joint JPDA: exact until gates overlap (multi-target milestone).
  - Pseudo-measurement update (z_pred + combined innovation) omits PDA's
    covariance-inflation "spread of innovations" term — optimistic in clutter.
  - Camera poses are per-step inputs (`camera_models`): the platform moves, so
    extrinsics cannot live in config. Fixed radar/lidar poses come from config.
  - Classification (decisions 012/015/D-B7): per cycle, each sensor's BEST
    associated class-carrying detection contributes a mass function; the cycle's
    masses fuse via conflict-weighted discounting, then Dempster-combine into
    the track's running DS-native belief (temporal fusion of prior with new
    evidence uses plain Dempster + the K-guard; discounting the prior too is a
    noted alternative).
"""

from __future__ import annotations

import logging

import numpy as np

from swarm_autonomy.edge.classification import DSClassifier
from swarm_autonomy.edge.config import TrackerConfig
from swarm_autonomy.edge.filters import initial_state, make_filter
from swarm_autonomy.edge.jpda import JPDA
from swarm_autonomy.edge.observation import CameraModel, h_camera, h_radar
from swarm_autonomy.edge.types import Detection, Track

log = logging.getLogger(__name__)

BIRTH_MAX_SPEED = 15.0  # m/s — max plausible target speed for two-point pairing


class MultiTargetTracker:
    def __init__(self, config: TrackerConfig) -> None:
        self.cfg = config
        self.filter = make_filter(config.ukf)
        self.jpda = JPDA(config.jpda)
        self.classifier = DSClassifier()
        self.tracks: list[Track] = []
        self._next_id: int = 0
        # R per sensor (config stores std-devs; R is the variance diagonal)
        self._R = {
            sid: np.diag(np.square(np.asarray(s.measurement_noise, dtype=float)))
            for sid, s in config.sensors.items()
        }
        # radar detections that matched no track last cycle, awaiting a second
        # sighting for two-point initiation: (position, sim-relative cycle index)
        self._birth_candidates: list[tuple[np.ndarray, int]] = []
        self._cycle = 0

    # ------------------------------------------------------------------ sensors
    def _observation_fn(self, sensor_id: str, camera_models: dict[str, CameraModel]):
        """Bind the sensor's h(x) for this cycle (pose-dependent for cameras)."""
        sensor = self.cfg.sensors[sensor_id]
        if sensor.sensor_type == "camera":
            cam = camera_models.get(sensor_id)
            if cam is None:
                raise ValueError(
                    f"camera '{sensor_id}' has detections but no CameraModel was "
                    f"passed to step() — camera extrinsics are per-step inputs."
                )
            return lambda x: h_camera(x, cam)
        if sensor.sensor_type == "radar":
            pos = np.asarray(sensor.position or [0.0, 0.0, 0.0])
            return lambda x: h_radar(x, pos)
        raise ValueError(f"unsupported sensor type: {sensor.sensor_type}")

    @staticmethod
    def _radar_to_position(sensor_pos: np.ndarray, z: np.ndarray) -> np.ndarray:
        """Invert [range, az, el, doppler] to a world position (spherical -> cartesian)."""
        rng, az, el = float(z[0]), float(z[1]), float(z[2])
        horiz = rng * np.cos(el)
        return sensor_pos + np.array([horiz * np.cos(az), horiz * np.sin(az), rng * np.sin(el)])

    # --------------------------------------------------------------------- step
    def step(
        self,
        detections: list[Detection],
        camera_models: dict[str, CameraModel] | None = None,
    ) -> list[Track]:
        """One tracker cycle. Returns the CONFIRMED tracks."""
        camera_models = camera_models or {}
        self._cycle += 1

        for track in self.tracks:
            track.state = self.filter.predict(track.state)

        by_sensor: dict[str, list[Detection]] = {}
        for det in detections:
            by_sensor.setdefault(det.sensor_id, []).append(det)

        associated_tracks: set[int] = set()
        unassociated_radar: list[Detection] = []
        cycle_masses: dict[int, list[dict[frozenset[str], float]]] = {}

        for sensor_id, dets in by_sensor.items():
            sensor = self.cfg.sensors.get(sensor_id)
            if sensor is None:
                log.warning("detections from unknown sensor '%s' ignored", sensor_id)
                continue
            h = self._observation_fn(sensor_id, camera_models)
            R = self._R[sensor_id]
            claimed: set[int] = set()  # detection indices gated by ANY track

            for track in self.tracks:
                z_pred, S = self.filter.measurement_prediction(track.state, h, R)
                gated_idx = self.jpda.gate(dets, z_pred, S, sensor.gate_threshold)
                claimed.update(gated_idx)
                if not gated_idx:
                    continue
                gated = [dets[i] for i in gated_idx]
                betas = self.jpda.compute_association_probabilities(gated, z_pred, S)
                combined = self.jpda.compute_combined_innovation(gated, betas, z_pred)
                track.state, _, _ = self.filter.update(track.state, z_pred + combined, h, R)
                # only count a real association (not null-dominated) as a hit
                if betas[1:].sum() > betas[0]:
                    associated_tracks.add(track.track_id)
                    # classification evidence: this sensor's most probable
                    # associated detection contributes one mass function
                    best = gated[int(np.argmax(betas[1:]))]
                    if best.class_label is not None:
                        cycle_masses.setdefault(track.track_id, []).append(
                            self.classifier.build_mass_function(
                                sensor.sensor_type, best.class_label, best.class_confidence
                            )
                        )

            if sensor.sensor_type == "radar":
                unassociated_radar.extend(d for i, d in enumerate(dets) if i not in claimed)

        # -------- classification fusion (012/015/D-B7) ---------------------
        for track in self.tracks:
            masses = cycle_masses.get(track.track_id)
            if not masses:
                continue
            new = self.classifier.combine_discounted(masses)
            track.class_beliefs = (
                self.classifier.combine(track.class_beliefs, new)
                if track.class_beliefs else new
            )

        # -------- lifecycle: hits / misses ---------------------------------
        for track in self.tracks:
            if track.track_id in associated_tracks:
                track.misses = 0
                track.age += 1
            else:
                track.misses += 1
        self.tracks = [t for t in self.tracks if t.misses < self.cfg.max_misses]

        # -------- birth: two-point radar initiation ------------------------
        self._initiate(unassociated_radar)

        return [t for t in self.tracks if t.age >= self.cfg.confirm_hits]

    # -------------------------------------------------------------------- birth
    def _initiate(self, unassociated_radar: list[Detection]) -> None:
        """Two-point differencing: pair an unmatched radar detection with one
        from the PREVIOUS cycle to get position + velocity (the init recipe the
        zero-velocity war story mandates — see explanations/edge/ukf.md)."""
        dt = self.cfg.ukf.dt
        new_candidates: list[tuple[np.ndarray, int]] = []
        for det in unassociated_radar:
            sensor = self.cfg.sensors[det.sensor_id]
            pos = self._radar_to_position(np.asarray(sensor.position or [0, 0, 0]), det.measurement)
            paired = False
            for prev_pos, prev_cycle in self._birth_candidates:
                if self._cycle - prev_cycle != 1:
                    continue
                vel = (pos - prev_pos) / dt
                if np.linalg.norm(vel) > BIRTH_MAX_SPEED:
                    continue
                x0 = np.concatenate([pos, vel, [0.5, 0.5, 0.5]])  # extent prior
                P0 = np.diag([0.25] * 3 + [1.0] * 3 + [0.25] * 3)
                self.tracks.append(
                    Track(track_id=self._new_track_id(),
                          state=initial_state(self.filter, x0, P0), age=1)
                )
                log.info("track %d initiated at %s", self.tracks[-1].track_id, pos.round(2))
                paired = True
                break
            if not paired:
                new_candidates.append((pos, self._cycle))
        self._birth_candidates = new_candidates

    def _new_track_id(self) -> int:
        tid = self._next_id
        self._next_id += 1
        return tid
