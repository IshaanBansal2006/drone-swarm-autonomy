"""Operator console — issues StructuredIntents to L2 (decision 030: Rerun shows
the picture; COMMANDS enter through this CLI, since a viewer hosts no buttons).

One-shot by design (scriptable, testable, demo-friendly):
    python3 -m mini_lattice.cop.console patrol --area 0,0 20,0 20,20 0,20
    python3 -m mini_lattice.cop.console track --track-id 0
    python3 -m mini_lattice.cop.console scan --area 0,0 30,0 30,30
    python3 -m mini_lattice.cop.console goto --point 5,5,2
"""

from __future__ import annotations

import argparse
import itertools
import time

import rclpy
from std_msgs.msg import String

from mini_lattice.schemas import StructuredIntent

_ids = itertools.count(int(time.time()) % 100000)


def parse_intent(argv: list[str] | None = None) -> StructuredIntent:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("verb", choices=["patrol", "track", "scan", "goto"])
    parser.add_argument("--area", nargs="+", help="polygon vertices x,y")
    parser.add_argument("--point", help="x,y,z")
    parser.add_argument("--track-id", type=int)
    parser.add_argument("--priority", type=int, default=0)
    args = parser.parse_args(argv)
    return StructuredIntent(
        intent_id=f"op-{next(_ids)}",
        verb=args.verb,
        area=[[float(v) for v in p.split(",")] for p in args.area] if args.area else None,
        point=[float(v) for v in args.point.split(",")] if args.point else None,
        target_track_id=args.track_id,
        priority=args.priority,
    )


def main() -> None:  # pragma: no cover — thin ROS wrapper
    intent = parse_intent()
    rclpy.init()
    node = rclpy.create_node("operator_console")
    pub = node.create_publisher(String, "/intent", 10)
    # A one-shot publisher must WAIT FOR DISCOVERY: with unicast initial-peers,
    # matching takes seconds — publishing before a subscriber matches drops the
    # message silently (observed live). Block until L2 is matched (or 15s).
    deadline = time.time() + 15.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if pub.get_subscription_count() == 0:
        raise SystemExit("no /intent subscriber found in 15s — is the L2 "
                         "mission node running?")
    for _ in range(3):  # small burst against reliability hiccups
        pub.publish(String(data=intent.model_dump_json()))
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"sent {intent.intent_id}: {intent.verb}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
