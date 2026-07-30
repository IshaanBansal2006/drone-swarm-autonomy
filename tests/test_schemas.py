"""Round-trip tests for the 040 cross-layer schemas (wire codec = JSON)."""

from __future__ import annotations

from swarm_autonomy.schemas import (
    EngagementProposal,
    StructuredIntent,
    TaskAssignment,
    TrackMsg,
    decode_mass,
    encode_mass,
)


def test_mass_codec_roundtrip() -> None:
    mass = {frozenset({"vehicle", "person"}): 0.6, frozenset({"aircraft"}): 0.1,
            frozenset({"vehicle", "person", "aircraft", "unknown"}): 0.3}
    assert decode_mass(encode_mass(mass)) == mass


def test_schemas_json_roundtrip() -> None:
    msgs = [
        TrackMsg(track_id=0, timestamp=12.3, position=[1, 2, 0.5], velocity=[0.5, 0.2, 0],
                 extent=[0.2, 0.2, 0.2], position_cov=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
                 class_label="vehicle", class_confidence=0.97,
                 class_beliefs=encode_mass({frozenset({"vehicle"}): 1.0}),
                 age=100, source_sensor_ids=["cam_front", "radar_gs"]),
        StructuredIntent(intent_id="i1", verb="patrol", area=[[0, 0], [10, 0], [10, 10]]),
        TaskAssignment(task_id="t1", intent_id="i1", drone_id="d1", task_type="goto_waypoint",
                       waypoints=[[1, 2, 3]]),
        EngagementProposal(proposal_id="p1", action="designate", target_track_id=0,
                           rationale="operator intent i1", deadline_s=10.0),
    ]
    for m in msgs:
        assert type(m).model_validate_json(m.model_dump_json()) == m
