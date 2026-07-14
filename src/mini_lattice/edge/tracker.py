"""Multi-target tracker — wires UKF + JPDA + DS classification."""

from __future__ import annotations

import logging

import numpy as np

from mini_lattice.edge.classification import DSClassifier
from mini_lattice.edge.config import TrackerConfig
from mini_lattice.edge.jpda import JPDA
from mini_lattice.edge.observation import h_camera
from mini_lattice.edge.types import Detection, Track, TrackState
from mini_lattice.edge.ukf import UKF

log = logging.getLogger(__name__)


class MultiTargetTracker:
    def __init__(self, config: TrackerConfig) -> None:
        self.cfg = config
        self.ukf = UKF(config.ukf)
        self.jpda = JPDA(config.jpda)
        self.classifier = DSClassifier()

        self.tracks: list[Track] = []
        self._next_id: int = 0

        self.R = np.diag(config.sensor.measurement_noise)

    def step(self, detections: list[Detection]) -> list[Track]:
        """One tracker cycle: predict -> associate -> update -> manage."""

        for track in self.tracks:
            track.state = self.ukf.predict(track.state)

        h = self._get_observation_fn(self.cfg.sensor.sensor_type)
        for track in self.tracks:
            z_pred = h(track.state.x)
            S = self._compute_innovation_covariance(track, h)

            gated_idx = self.jpda.gate(track, detections, z_pred, S)
            if not gated_idx:
                track.misses += 1
                continue

            gated_dets = [detections[i] for i in gated_idx]
            betas = self.jpda.compute_association_probabilities(
                track.state, gated_dets, z_pred, S,
            )
            combined_innov = self.jpda.compute_combined_innovation(
                gated_dets, betas, z_pred,
            )

            track.state, _, _ = self.ukf.update(
                track.state, z_pred + combined_innov, h, self.R,
            )
            track.misses = 0
            track.age += 1

            best_det_idx = int(np.argmax(betas[1:])) if len(betas) > 1 else None
            if best_det_idx is not None:
                det = gated_dets[best_det_idx]
                if det.class_label is not None:
                    sensor_mass = self.classifier.build_mass_function(
                        self.cfg.sensor.sensor_type, det.class_label, det.class_confidence,
                    )
                    if track.class_beliefs:
                        existing_mass = track.class_beliefs  # type: ignore[arg-type]
                        track.class_beliefs = self.classifier.combine(existing_mass, sensor_mass)  # type: ignore[assignment]
                    else:
                        track.class_beliefs = sensor_mass  # type: ignore[assignment]

        self._manage_tracks(detections)
        return [t for t in self.tracks if t.age >= self.cfg.confirm_hits]

    def _get_observation_fn(self, sensor_type: str):  # noqa: ANN202
        if sensor_type == "camera":
            return h_camera
        raise ValueError(f"Unknown sensor type: {sensor_type}")

    def _compute_innovation_covariance(self, track: Track, h) -> np.ndarray:  # noqa: ANN001
        # Placeholder — wire to UKF sigma-point transform once update() is implemented
        return self.R.copy()

    def _manage_tracks(self, detections: list[Detection]) -> None:
        self.tracks = [t for t in self.tracks if t.misses < self.cfg.max_misses]
        # TODO(ishaan): initiate new tracks from unassociated detections

    def _new_track_id(self) -> int:
        tid = self._next_id
        self._next_id += 1
        return tid
