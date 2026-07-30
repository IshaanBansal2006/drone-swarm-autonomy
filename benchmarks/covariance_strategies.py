"""D-B2 bake-off: covariance strategies under the full camera+radar fusion loop.

Compares three ways of keeping the UKF covariance healthy:
  - shortcut : P+ = P - K S K^T, symmetrize, eigenvalue-floor repair on violation
  - joseph   : quadratic-in-K form, same guards
  - srukf    : square-root UKF (factor carried; cannot go indefinite; downdate
               failures repaired + counted)

Two scenarios x N seeds:
  - nominal : thin-slice-like init (good prior)
  - stress  : wide/bad prior (P0 = 4I, position off by ~3 sigma) — the regime
              that caused the live 2026-07-29 failures

Metrics per strategy: PSD repairs triggered, RMSE (pos/vel/extent) over the
last 100 steps, mean NEES (consistency: E[(x-xhat)^T P^-1 (x-xhat)] ~ n = 9;
<< n overconfident-inverse, >> n overconfident), wall-clock us/step.

Run:  PYTHONPATH=src .venv/bin/python benchmarks/covariance_strategies.py
"""

from __future__ import annotations

import time

import numpy as np

from mini_lattice.edge.config import UKFConfig
from mini_lattice.edge.observation import CameraModel, h_camera, h_radar
from mini_lattice.edge.srukf import SquareRootUKF, SRTrackState
from mini_lattice.edge.types import TrackState
from mini_lattice.edge.ukf import UKF

TRUE_EXTENT = np.array([0.2, 0.2, 0.2])
VEL = np.array([0.5, 0.2, 0.0])
DT = 0.1
STEPS = 300
SEEDS = 20
R_CAMERA = np.diag([2.0**2, 2.0**2, 3.0**2, 3.0**2])
R_RADAR = np.diag([0.1**2, 0.01**2, 0.01**2, 0.1**2])
RADAR_POS = np.zeros(3)


def chase_camera(target_pos: np.ndarray) -> CameraModel:
    cam_pos = target_pos + np.array([-6.0, -2.0, 3.0])
    z_c = (target_pos - cam_pos) / np.linalg.norm(target_pos - cam_pos)
    x_c = np.cross(z_c, [0.0, 0.0, 1.0])
    x_c /= np.linalg.norm(x_c)
    return CameraModel(fx=480.0, fy=480.0, cx=480.0, cy=300.0, width=960, height=600,
                       R_wc=np.stack([x_c, np.cross(z_c, x_c), z_c]), t_w=cam_pos)


def make_filter(strategy: str):
    cfg = UKFConfig(covariance_form="joseph" if strategy == "joseph" else "shortcut")
    return SquareRootUKF(cfg) if strategy == "srukf" else UKF(cfg)


def init_state(rng: np.random.Generator, pos0: np.ndarray, stress: bool):
    if stress:
        x0 = np.concatenate([pos0 + rng.normal(0, 2.0, 3), np.zeros(3), [1.0, 1.0, 1.0]])
        P0 = np.diag([4.0] * 9)
    else:
        x0 = np.concatenate([pos0 + rng.normal(0, 0.5, 3), VEL + rng.normal(0, 0.3, 3),
                             [0.5, 0.5, 0.5]])
        P0 = np.diag([0.25] * 9)
    return x0, P0


def run_once(strategy: str, seed: int, stress: bool) -> dict:
    rng = np.random.default_rng(seed)
    filt = make_filter(strategy)
    pos0 = np.array([0.0, 0.0, 0.5])
    x0, P0 = init_state(rng, pos0, stress)
    if strategy == "srukf":
        state = SRTrackState(x=x0, S=np.linalg.cholesky(P0))
    else:
        state = TrackState(x=x0, P=P0)

    errs, nees = [], []
    t0 = time.perf_counter()
    for k in range(1, STEPS + 1):
        pos = pos0 + VEL * (k * DT)
        x_true = np.concatenate([pos, VEL, TRUE_EXTENT])
        cam = chase_camera(pos)
        z_c = h_camera(x_true, cam) + rng.multivariate_normal(np.zeros(4), R_CAMERA)
        z_r = h_radar(x_true, RADAR_POS) + rng.multivariate_normal(np.zeros(4), R_RADAR)

        state = filt.predict(state)
        state, _, _ = filt.update(state, z_r, lambda x: h_radar(x, RADAR_POS), R_RADAR)
        state, _, _ = filt.update(state, z_c, lambda x: h_camera(x, cam), R_CAMERA)

        if k > STEPS - 100:
            e = state.x - x_true
            P = state.S @ state.S.T if strategy == "srukf" else state.P
            errs.append(e)
            nees.append(float(e @ np.linalg.solve(P, e)))
    dt_us = (time.perf_counter() - t0) / STEPS * 1e6

    E = np.array(errs)
    return {
        "repairs": filt.repairs,
        "rmse_pos": float(np.sqrt(np.mean(np.sum(E[:, 0:3] ** 2, axis=1)))),
        "rmse_vel": float(np.sqrt(np.mean(np.sum(E[:, 3:6] ** 2, axis=1)))),
        "rmse_ext": float(np.sqrt(np.mean(np.sum(E[:, 6:9] ** 2, axis=1)))),
        "nees": float(np.mean(nees)),
        "us_step": dt_us,
    }


def main() -> None:
    for stress in (False, True):
        name = "STRESS (wide/bad prior)" if stress else "NOMINAL (thin-slice-like)"
        print(f"\n=== {name} — {SEEDS} seeds x {STEPS} steps, camera+radar fusion ===")
        print(f"{'strategy':>9} | {'repairs':>7} | {'rmse_pos':>8} | {'rmse_vel':>8} | "
              f"{'rmse_ext':>8} | {'NEES(~9)':>8} | {'us/step':>7}")
        for strategy in ("shortcut", "joseph", "srukf"):
            agg = {k: [] for k in ("repairs", "rmse_pos", "rmse_vel", "rmse_ext", "nees", "us_step")}
            for seed in range(SEEDS):
                r = run_once(strategy, seed, stress)
                for k, v in r.items():
                    agg[k].append(v)
            print(f"{strategy:>9} | {int(np.sum(agg['repairs'])):>7} | "
                  f"{np.mean(agg['rmse_pos']):>8.3f} | {np.mean(agg['rmse_vel']):>8.3f} | "
                  f"{np.mean(agg['rmse_ext']):>8.3f} | {np.mean(agg['nees']):>8.1f} | "
                  f"{np.mean(agg['us_step']):>7.0f}")


if __name__ == "__main__":
    main()
