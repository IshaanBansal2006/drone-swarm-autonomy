"""Train the localization-aware CBBA score (decision 024) and write the weights
the mission node loads.

    python benchmarks/learned_score.py [--iters 4] [--missions 8] [--horizon 60]

Writes config/learned_score.json (weights + predictor) and
config/learned_score_report.json (per-iteration held-out returns, convergence
rate, weights) — the two numbers 024 says the extension must carry.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import time
from pathlib import Path

from swarm_autonomy.autonomy.training import TrainConfig, train
from swarm_autonomy.scene import demo_scene

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=4)
    ap.add_argument("--missions", type=int, default=8)
    ap.add_argument("--holdout", type=int, default=4)
    ap.add_argument("--horizon", type=float, default=60.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=ROOT / "config" / "learned_score.json")
    args = ap.parse_args()
    cfg = TrainConfig(max_iters=args.iters, train_missions=args.missions,
                      holdout_missions=args.holdout, horizon_s=args.horizon, seed=args.seed)
    t0 = time.time()
    score, report = train(demo_scene(), cfg)
    score.to_json(args.out)
    args.out.with_name("learned_score_report.json").write_text(json.dumps({
        "config": dataclasses.asdict(cfg), "wall_s": round(time.time() - t0, 1),
        "report": dataclasses.asdict(report)}, indent=2))
    print(f"stopped: {report.stopped_because}; weights -> {args.out} ({time.time() - t0:.0f}s)")


if __name__ == "__main__":
    main()
