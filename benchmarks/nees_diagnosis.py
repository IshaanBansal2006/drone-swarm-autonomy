"""D-B11: consistency diagnosis + Q_ext sweep (SR-UKF, nominal scenario).

The bake-off found NEES ~ 9,800 vs the ideal ~9 (a consistent filter's
normalized estimation error squared should average the state dimension).
Hypothesis: extent overconfidence — Q_ext = 1e-8 collapses the extent
covariance while the estimate is still biased ("smug filter"), so tiny claimed
variance divides a not-tiny error.

This script (a) splits NEES into position / velocity / extent blocks (using the
marginal 3x3 covariance blocks) to locate the inconsistency, and (b) sweeps
Q_ext to find a value that restores plasticity without drowning the extent
estimate in noise.

Run:  PYTHONPATH=src .venv/bin/python benchmarks/nees_diagnosis.py
"""

from __future__ import annotations

import numpy as np

from swarm_autonomy.edge.config import UKFConfig
from swarm_autonomy.edge.filters import initial_state, make_filter
from swarm_autonomy.edge.observation import CameraModel, h_camera, h_radar

TRUE_EXTENT = np.array([0.2, 0.2, 0.2])
VEL = np.array([0.5, 0.2, 0.0])
DT = 0.1
STEPS = 300
SEEDS = 20
R_CAMERA = np.diag([2.0**2, 2.0**2, 3.0**2, 3.0**2])
R_RADAR = np.diag([0.1**2, 0.01**2, 0.01**2, 0.1**2])
RADAR_POS = np.zeros(3)
BLOCKS = {"pos": slice(0, 3), "vel": slice(3, 6), "ext": slice(6, 9)}


def chase_camera(target_pos: np.ndarray) -> CameraModel:
    cam_pos = target_pos + np.array([-6.0, -2.0, 3.0])
    z_c = (target_pos - cam_pos) / np.linalg.norm(target_pos - cam_pos)
    x_c = np.cross(z_c, [0.0, 0.0, 1.0])
    x_c /= np.linalg.norm(x_c)
    return CameraModel(fx=480.0, fy=480.0, cx=480.0, cy=300.0, width=960, height=600,
                       R_wc=np.stack([x_c, np.cross(z_c, x_c), z_c]), t_w=cam_pos)


def run_once(q_ext: float, seed: int) -> dict:
    rng = np.random.default_rng(seed)
    cfg = UKFConfig(process_noise=[1e-3] * 3 + [1e-2] * 3 + [q_ext] * 3)
    filt = make_filter(cfg)
    pos0 = np.array([0.0, 0.0, 0.5])
    x0 = np.concatenate([pos0 + rng.normal(0, 0.5, 3), VEL + rng.normal(0, 0.3, 3),
                         [0.5, 0.5, 0.5]])
    state = initial_state(filt, x0, np.diag([0.25] * 9))

    out = {k: [] for k in ("nees_pos", "nees_vel", "nees_ext", "rmse_ext", "rmse_pos")}
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
            P = state.S @ state.S.T
            e = state.x - x_true
            for name, sl in BLOCKS.items():
                out[f"nees_{name}"].append(float(e[sl] @ np.linalg.solve(P[sl, sl], e[sl])))
            out["rmse_ext"].append(float(np.sum(e[6:9] ** 2)))
            out["rmse_pos"].append(float(np.sum(e[0:3] ** 2)))
    return {k: float(np.mean(v)) for k, v in out.items()}


def main() -> None:
    print(f"SR-UKF, nominal scenario, {SEEDS} seeds x {STEPS} steps; block-NEES ideal ~ 3.0")
    print(f"{'Q_ext':>8} | {'NEES_pos':>8} | {'NEES_vel':>8} | {'NEES_ext':>9} | "
          f"{'rmse_ext':>8} | {'rmse_pos':>8}")
    for q_ext in (1e-8, 1e-6, 1e-5, 1e-4, 1e-3):
        agg = {k: [] for k in ("nees_pos", "nees_vel", "nees_ext", "rmse_ext", "rmse_pos")}
        for seed in range(SEEDS):
            r = run_once(q_ext, seed)
            for k, v in r.items():
                agg[k].append(v)
        print(f"{q_ext:>8.0e} | {np.mean(agg['nees_pos']):>8.2f} | "
              f"{np.mean(agg['nees_vel']):>8.2f} | {np.mean(agg['nees_ext']):>9.1f} | "
              f"{np.sqrt(np.mean(agg['rmse_ext'])):>8.3f} | "
              f"{np.sqrt(np.mean(agg['rmse_pos'])):>8.3f}")


if __name__ == "__main__":
    main()
