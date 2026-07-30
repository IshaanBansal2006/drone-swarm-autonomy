"""L2 world state — the bookkeeping every autonomy component reads (decision 020).

Holds what L2 knows: the drone fleet (poses, batteries, capabilities) and the
live track picture from L1 (TrackMsg feed). Deliberately dumb: no inference, no
prediction — estimation belongs to L1, decisions to the decomposer/allocator.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from mini_lattice.schemas import TrackMsg


@dataclass
class DroneState:
    """One platform's status as known to L2."""

    drone_id: str
    position: list[float]  # [x, y, z] m, world frame
    velocity: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    battery: float = 1.0  # 0..1
    speed: float = 2.0  # nominal cruise speed m/s (allocator travel-time model)
    capabilities: frozenset[str] = frozenset()  # e.g. {"camera", "radar"}
    available: bool = True


class WorldState:
    """Aggregated L2 picture: fleet + tracks, timestamped by the newest input."""

    def __init__(self) -> None:
        self.drones: dict[str, DroneState] = {}
        self.tracks: dict[int, TrackMsg] = {}
        self.time: float = 0.0

    def update_drone(self, drone: DroneState) -> None:
        self.drones[drone.drone_id] = drone

    def update_tracks(self, tracks: list[TrackMsg], timestamp: float) -> None:
        """Replace the track picture (L1 publishes complete confirmed sets)."""
        self.tracks = {t.track_id: t for t in tracks}
        self.time = max(self.time, timestamp)

    def available_drones(self) -> list[DroneState]:
        return [d for d in self.drones.values() if d.available and d.battery > 0.1]
