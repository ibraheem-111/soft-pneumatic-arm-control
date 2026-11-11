"""The lab's recorded tip trajectories (axial, circular, triangular) as tracking references.

Read at runtime from the upstream repo (Jeevan-HM/Soft-Robotic-Arm, branch `main`,
`data/_data_extract/`) with `git show`; nothing is copied into this repo. Chosen runs match
make_calibrated_sim()'s defaults (parallel plumbing, Segment-1 charge 2 psi, 5 psi peak), all at
0.1 Hz. The active window starts when Segment 1 is isolated (its desired pressure drops to ~0).

Tip = rigid body 3 minus rigid body 2, rotated into rigid body 1's frame, OptiTrack Y-up
converted to MuJoCo Z-up as [x, -z, y], minus its value at the start of the window, in mm;
resampled to 100 Hz and smoothed zero-phase (Butterworth, 4th order, 1 Hz). One yaw angle,
fitted once pooled over the three runs against an open-loop replay of the recorded commands,
maps the mocap frame to the simulator frame. Used by scripts/benchmark_all_tasks.py and
scripts/benchmark_unified.py.
"""

import io
import subprocess
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt

from soft_robotic_arm import make_calibrated_sim

from softarm import constants

HZ = int(constants.HZ)         # samples per second, as an int (grids, sim rate)
SIM_SEED = 0
UPSTREAM = constants.ROOT.parent / "Soft-Robotic-Arm"
UPSTREAM_REV = "origin/main"
RUNS = {
    "axial": "data/_data_extract/parallel/axial_2-5_parallel.csv",
    "circular": "data/_data_extract/parallel/circular_2-5_parallel.csv",
    "triangular": "data/_data_extract/parallel/triangular_2-5_parallel.csv",
}
PRE_S = 5.0            # recording kept before the active window (filter padding)
CUTOFF_HZ = 1.0
RECORDED_CHARGE_PSI = 2.0  # S2-S4 held here before the active window in the recording


def _rotation_xyzw(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_run(upstream: Path, path: str, track_s: float) -> dict:
    """Commands (S2-S4, psi) and tip displacement (mm, Z-up, mocap yaw) on a 100 Hz grid."""
    text = subprocess.run(["git", "-C", str(upstream), "show", f"{UPSTREAM_REV}:{path}"],
                          check=True, capture_output=True, text=True).stdout
    a = np.genfromtxt(io.StringIO(text), delimiter=",", names=True)
    t = a["time"]
    d1 = a["Desired_pressure_segment_1"]
    first = np.flatnonzero(d1 >= 0.5 * RECORDED_CHARGE_PSI)[0]
    start = first + np.flatnonzero(d1[first:] <= 0.1)[0]
    grid = t[start] + np.arange(-int(PRE_S * HZ), int(track_s * HZ) + 1) / HZ
    cmd = np.stack([np.interp(grid, t, a[f"Desired_pressure_segment_{s}"]) for s in (2, 3, 4)], 1)

    rb = {k: np.stack([a[f"Rigid_body_{k}_{c}"] for c in "xyz"], 1) for k in (2, 3)}
    q1 = np.stack([a[f"Rigid_body_1_q{c}"] for c in "xyzw"], 1)
    mt = a["mocap_time_rel_s"]
    ok = (np.isfinite(mt) & np.all(np.isfinite(rb[2]) & np.isfinite(rb[3]), 1)
          & np.all(np.isfinite(q1), 1))
    mt_u, idx = np.unique(mt[ok], return_index=True)   # mocap rows repeat at ~50 Hz
    arm = (rb[3] - rb[2])[ok][idx]
    arm = np.array([_rotation_xyzw(q).T @ v for q, v in zip(q1[ok][idx], arm)])
    arm = np.stack([np.interp(grid, mt_u, arm[:, j]) for j in range(3)], 1)
    tip = 1000.0 * np.stack([arm[:, 0], -arm[:, 2], arm[:, 1]], 1)   # Y-up -> Z-up
    b, a_f = butter(4, CUTOFF_HZ / (HZ / 2))
    smooth = filtfilt(b, a_f, tip, axis=0)
    i0 = int(PRE_S * HZ)
    return dict(cmd=cmd[i0:], disp=smooth[i0:] - smooth[i0], disp_raw=tip[i0:] - smooth[i0],
                active_start_s=float(t[start]))


def replay(cmd: np.ndarray, seed: int = SIM_SEED, settle_s: float = 10.0) -> np.ndarray:
    """Open-loop replay of recorded commands; tip displacement (mm) from the 2 psi settled pose."""
    sim = make_calibrated_sim(control_hz=HZ, seed=seed)
    sim.reset()
    for _ in range(int(settle_s * HZ)):
        obs = sim.step(np.full(3, RECORDED_CHARGE_PSI))
    home = obs["tip_pos"].copy()
    out = np.empty((len(cmd) - 1, 3))
    for k, c in enumerate(cmd[:-1]):   # same indexing as the tracking loop
        out[k] = 1000.0 * (sim.step(np.clip(c, 0.0, constants.LIMIT_PSI))["tip_pos"] - home)
    sim.close()
    return out


def fit_yaw(recorded: list[np.ndarray], simulated: list[np.ndarray]) -> float:
    """Proper 2-D rotation (Procrustes) taking mean-removed recorded xy onto simulated xy."""
    A = np.vstack([r[:, :2] - r[:, :2].mean(0) for r in recorded])
    B = np.vstack([s[:, :2] - s[:, :2].mean(0) for s in simulated])
    U, _, Vt = np.linalg.svd(A.T @ B)
    U[:, 1] *= np.sign(np.linalg.det(U @ Vt))
    R = (U @ Vt).T
    return float(np.arctan2(R[1, 0], R[0, 0]))


def yaw_matrix(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def recorded_runs(upstream: Path, track_s: float) -> tuple[dict, dict, float]:
    """Load the three runs, replay their commands open loop, fit the pooled mocap yaw (rad).

    The reference for waveform w in the simulator frame is runs[w]["disp"] @ yaw_matrix(psi).T.
    """
    runs = {w: load_run(upstream, p, track_s) for w, p in RUNS.items()}
    replays = {w: replay(r["cmd"]) for w, r in runs.items()}
    psi = fit_yaw([runs[w]["disp"][1:] for w in RUNS], [replays[w] for w in RUNS])
    return runs, replays, psi
