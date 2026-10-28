"""Benchmark every controller on the lab's three recorded waveforms: axial, circular, triangular.

Each task is to track the real robot's recorded tip trajectory for one waveform, from the
lab's calibration experiments (upstream repo, branch `main`, `data/_data_extract/`, read at
runtime with `git show`; nothing is copied into this repo). Chosen runs match
`make_calibrated_sim()`'s defaults (parallel plumbing, Segment-1 charge 2 psi); the 5 psi
peak is used because the 10 psi runs command up to 10 psi, above the 0-9 psi limit:
    parallel/axial_2-5_parallel.csv       (only S3 driven, 0-5 psi)
    parallel/circular_2-5_parallel.csv    (S2-S4, 0-5 psi)
    parallel/triangular_2-5_parallel.csv  (S2-S4, 0-5 psi)
All at 0.1 Hz. The active window is the 180 s after Segment 1 is isolated (its desired
pressure drops to ~0), as in upstream's former `mujoco/experiment_data.py` (commit a654f09).

Reference: tip = rigid body 3 minus rigid body 2, rotated into rigid body 1's frame,
OptiTrack Y-up converted to MuJoCo Z-up as [x, -z, y] (upstream
`mujoco/docs/hardware_integration.md`), minus its value at the start of the window, in mm.
Mocap (~50 Hz, ~0.05-0.1 mm jitter) is resampled to 100 Hz and smoothed zero-phase
(Butterworth, 4th order, 1 Hz), because the controllers differentiate the reference. One
yaw angle maps the mocap frame to the simulator frame; it is fitted once, pooled over the
three runs, between the recorded motion and an open-loop replay of the recorded commands
in the simulator (both mean-removed).

Evaluation mirrors `soft_robotic_arm.evaluation.evaluate_controller`: make_calibrated_sim
(control_hz=100, seed=0), settle 5 s at 4.5 psi on S2-S4, home = settled tip, then 180 s of
tracking `home + recorded displacement(t)`; the tip after each step is scored against the
reference passed to that step. Metrics: planar (xy) and axial (z) RMSE over the whole run
and after the first 5 s, and the share of command samples at 0 or 9 psi. Controllers and
gains are those of scripts/compare_controllers.py (gains tuned on the scored circle, not
here). The PD and notebook-SMC controllers command only x-y; z is uncontrolled for them.

Reachability is reported two ways: (1) open-loop replay of the recorded commands (settle
10 s at 2 psi, as in the recording), sim vs recorded tip; (2) the share of reference
samples whose static command G^-1 X (G = M1 static gain, docs/figures/task_space_model.json)
leaves the +-4.5 psi range around the 4.5 psi baseline, for xyz and for xy alone.

Writes docs/figures/benchmark_all_tasks.json.

Usage:
    uv run python scripts/benchmark_all_tasks.py [--upstream PATH] [--track-s 180]
"""

import argparse
import io
import json
import subprocess
import time
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt

import compare_controllers as C
from soft_robotic_arm import make_calibrated_sim
from soft_robotic_arm.evaluation import CircularTrackingTask, evaluate_controller, make_reference

ROOT = Path(__file__).resolve().parent.parent
FIG = ROOT / "docs" / "figures"
UPSTREAM = ROOT.parent / "Soft-Robotic-Arm"
UPSTREAM_REV = "origin/main"
RUNS = {
    "axial": "data/_data_extract/parallel/axial_2-5_parallel.csv",
    "circular": "data/_data_extract/parallel/circular_2-5_parallel.csv",
    "triangular": "data/_data_extract/parallel/triangular_2-5_parallel.csv",
}
HZ = 100
SIM_SEED = 0
SETTLE_S = 5.0
BASELINE_PSI = 4.5
LIMIT_PSI = 9.0
PRE_S = 5.0            # recording kept before the active window (filter padding)
CUTOFF_HZ = 1.0
RECORDED_CHARGE_PSI = 2.0  # S2-S4 held here before the active window in the recording


# ------------------------------------------------------------------------ recorded data

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
    for k, c in enumerate(cmd[:-1]):   # same indexing as the tracking loop below
        out[k] = 1000.0 * (sim.step(np.clip(c, 0.0, LIMIT_PSI))["tip_pos"] - home)
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


# --------------------------------------------------------------------------- tracking

def track(controller, ref_fn, n_steps: int, seed: int = SIM_SEED) -> dict:
    """evaluate_controller's loop with an arbitrary reference ref_fn(home, k) -> (3,) m."""
    sim = make_calibrated_sim(control_hz=HZ, seed=seed)
    limit = min(LIMIT_PSI, float(sim.cfg.p_max))
    baseline = np.full(3, BASELINE_PSI)
    obs = sim.reset()
    for _ in range(int(round(SETTLE_S * HZ))):
        obs = sim.step(baseline)
    home = np.asarray(obs["tip_pos"], float).copy()
    tips = np.empty((n_steps, 3))
    refs = np.empty((n_steps, 3))
    cmds = np.empty((n_steps, 3))
    for k in range(n_steps):
        ref = ref_fn(home, k)
        cmd = np.asarray(controller.compute(float(obs["time"]), obs, ref), float)
        if cmd.shape != (3,) or not np.all(np.isfinite(cmd)):
            raise ValueError("controller must return 3 finite S2-S4 setpoints")
        cmd = np.clip(cmd, 0.0, limit)
        obs = sim.step(cmd)
        tips[k], refs[k], cmds[k] = obs["tip_pos"], ref, cmd
    sim.close()
    return dict(tips=tips, refs=refs, cmds=cmds, home=home)


def metrics(err_mm: np.ndarray, cmds: np.ndarray | None = None, after_s: float = 5.0) -> dict:
    """err_mm (N, 3) = tip - reference. Planar and axial RMSE, whole run and after `after_s`."""
    late = np.arange(len(err_mm)) / HZ >= after_s
    planar = np.linalg.norm(err_mm[:, :2], axis=1)
    out = dict(planar_rmse_mm=float(np.sqrt(np.mean(planar**2))),
               planar_rmse_after_5s_mm=float(np.sqrt(np.mean(planar[late] ** 2))),
               z_rmse_mm=float(np.sqrt(np.mean(err_mm[:, 2] ** 2))),
               z_rmse_after_5s_mm=float(np.sqrt(np.mean(err_mm[late, 2] ** 2))))
    if cmds is not None:
        out["saturated_pct"] = float(100 * np.mean((cmds <= 0.0) | (cmds >= LIMIT_PSI)))
    return out


def static_infeasible_pct(disp_mm: np.ndarray) -> dict:
    """Share of samples whose static command G^-1 X leaves +-4.5 psi (xyz; xy via pinv)."""
    G = np.array(json.loads((FIG / "task_space_model.json").read_text())["M1"]["G"])
    span = min(BASELINE_PSI, LIMIT_PSI - BASELINE_PSI)
    u3 = disp_mm @ np.linalg.inv(G).T
    u2 = disp_mm[:, :2] @ np.linalg.pinv(G[:2]).T
    return dict(xyz_pct=float(100 * np.mean(np.any(np.abs(u3) > span, 1))),
                xy_pct=float(100 * np.mean(np.any(np.abs(u2) > span, 1))))


def check_harness() -> float:
    """|RMSE difference| between track() and evaluate_controller on the scored circle."""
    task = CircularTrackingTask()
    name, _, make = C.controllers()[-1]
    sim = make_calibrated_sim(control_hz=HZ, seed=SIM_SEED)
    ref = evaluate_controller(make(), task=task, sim=sim).rmse_mm
    sim.close()
    r = track(make(), lambda home, k: make_reference(home, k / HZ, task), task.track_steps)
    mine = metrics(1000.0 * (r["tips"] - r["refs"]))["planar_rmse_mm"]
    return abs(mine - ref)


# ------------------------------------------------------------------------------ main

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--upstream", type=Path, default=UPSTREAM, help="clone of Jeevan-HM/Soft-Robotic-Arm")
    ap.add_argument("--track-s", type=float, default=180.0)
    args = ap.parse_args()
    n = int(round(args.track_s * HZ))
    t_start = time.perf_counter()

    print(f"harness check (scored circle, {C.controllers()[-1][0]}): "
          f"|track - evaluate_controller| = {check_harness():.2e} mm")

    runs, replays, psi = recorded_runs(args.upstream, args.track_s)
    Rz = yaw_matrix(psi)
    print(f"mocap -> sim yaw: {np.degrees(psi):.1f} deg (pooled fit, 3 runs)")

    out = {"runs": RUNS, "upstream_rev": UPSTREAM_REV, "sim_seed": SIM_SEED, "track_s": args.track_s,
           "yaw_deg": float(np.degrees(psi)), "smoothing": f"Butterworth 4th order {CUTOFF_HZ} Hz, filtfilt",
           "tasks": {}}
    for w, r in runs.items():
        disp = r["disp"] @ Rz.T
        raw = r["disp_raw"] @ Rz.T
        rec = disp[1:n + 1]          # the reference passed at step k is scored after the step
        info = dict(
            reference_range_mm=dict(min=disp[:n].min(0).tolist(), max=disp[:n].max(0).tolist()),
            smoothing_rms_mm=np.sqrt(np.mean((raw - disp) ** 2, 0)).tolist(),
            open_loop_replay=metrics(replays[w][:n] - rec),
            static_infeasible=static_infeasible_pct(disp[:n]),
            controllers={},
        )
        out["tasks"][w] = info
        ref_fn = lambda home, k, d=disp: home + d[k] / 1000.0  # noqa: E731
        for name, _, make in C.controllers():
            t0 = time.perf_counter()
            res = track(make(), ref_fn, n)
            row = metrics(1000.0 * (res["tips"] - res["refs"]), res["cmds"])
            row["run_s"] = time.perf_counter() - t0
            info["controllers"][name] = row
        print(f"  {w}: done ({time.perf_counter() - t_start:.0f} s elapsed)")

    names = [nm for nm, _, _ in C.controllers()]
    print(f"\nmake_calibrated_sim seed {SIM_SEED}, {args.track_s:g} s tracking. Planar (xy) and "
          "axial (z) RMSE in mm, whole run / after 5 s; sat = % of commands at 0 or 9 psi")
    for w, info in out["tasks"].items():
        print(f"\n{w.upper()}  ({RUNS[w]})")
        rg = info["reference_range_mm"]
        print(f"  reference range x {rg['min'][0]:.1f}..{rg['max'][0]:.1f}, y {rg['min'][1]:.1f}.."
              f"{rg['max'][1]:.1f}, z {rg['min'][2]:.1f}..{rg['max'][2]:.1f} mm; smoothing removed "
              f"{np.round(info['smoothing_rms_mm'], 3).tolist()} mm RMS")
        o = info["open_loop_replay"]
        print(f"  open-loop replay of recorded commands: xy {o['planar_rmse_mm']:.2f} / "
              f"{o['planar_rmse_after_5s_mm']:.2f}, z {o['z_rmse_mm']:.2f} / {o['z_rmse_after_5s_mm']:.2f}")
        s = info["static_infeasible"]
        print(f"  static command outside 0-9 psi: {s['xyz_pct']:.1f} % of samples (xyz), "
              f"{s['xy_pct']:.1f} % (xy only)")
        print(f"  {'controller':<60}{'xy':>7}{'xy>5s':>7}{'z':>7}{'z>5s':>7}{'sat%':>7}{'run s':>7}")
        for nm in names:
            m = info["controllers"][nm]
            print(f"  {nm:<60}{m['planar_rmse_mm']:>7.2f}{m['planar_rmse_after_5s_mm']:>7.2f}"
                  f"{m['z_rmse_mm']:>7.2f}{m['z_rmse_after_5s_mm']:>7.2f}{m['saturated_pct']:>7.1f}"
                  f"{m['run_s']:>7.1f}")
    out["total_run_s"] = time.perf_counter() - t_start
    (FIG / "benchmark_all_tasks.json").write_text(json.dumps(out, indent=2))
    print(f"\ntotal {out['total_run_s']:.0f} s; wrote {FIG / 'benchmark_all_tasks.json'}")


if __name__ == "__main__":
    main()
