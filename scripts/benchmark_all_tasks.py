"""Benchmark every controller on the lab's three recorded waveforms: axial, circular, triangular.

Each task is to track the real robot's recorded tip trajectory for one waveform, from the
lab's calibration experiments (softarm.evaluation.recorded; upstream repo, branch `main`, `data/_data_extract/`, read at
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

Evaluation is softarm.evaluation.track, which mirrors
`soft_robotic_arm.evaluation.evaluate_controller`: make_calibrated_sim
(control_hz=100, seed=0), settle 5 s at 4.5 psi on S2-S4, home = settled tip, then 180 s of
tracking `home + recorded displacement(t)`; the tip after each step is scored against the
reference passed to that step. Metrics: planar (xy) and axial (z) RMSE over the whole run
and after the first 5 s, and the share of command samples at 0 or 9 psi. Controllers and
gains are softarm.controllers.presets.baseline_set(), as in scripts/compare_controllers.py
(gains tuned on the scored circle, not here). The PD and notebook-SMC controllers command only x-y; z is uncontrolled for them.

Reachability is reported two ways: (1) open-loop replay of the recorded commands (settle
10 s at 2 psi, as in the recording), sim vs recorded tip; (2) the share of reference
samples whose static command G^-1 X (G = M1 static gain, docs/figures/task_space_model.json)
leaves the +-4.5 psi range around the 4.5 psi baseline, for xyz and for xy alone.

Writes docs/figures/benchmark_all_tasks.json.

Usage:
    uv run python scripts/benchmark_all_tasks.py [--upstream PATH] [--track-s 180]
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from soft_robotic_arm.evaluation import CircularTrackingTask, make_reference

from softarm.constants import BASE_PSI, FIG, LIMIT_PSI, TASK_SPACE_JSON
from softarm.controllers.presets import baseline_set
from softarm.evaluation import evaluate, planar_metrics, track
from softarm.evaluation.recorded import (CUTOFF_HZ, HZ, RUNS, SIM_SEED, UPSTREAM, UPSTREAM_REV,
                                         recorded_runs, yaw_matrix)


def static_infeasible_pct(disp_mm: np.ndarray) -> dict:
    """Share of samples whose static command G^-1 X leaves +-4.5 psi (xyz; xy via pinv)."""
    G = np.array(json.loads(TASK_SPACE_JSON.read_text())["M1"]["G"])
    span = min(BASE_PSI, LIMIT_PSI - BASE_PSI)
    u3 = disp_mm @ np.linalg.inv(G).T
    u2 = disp_mm[:, :2] @ np.linalg.pinv(G[:2]).T
    return dict(xyz_pct=float(100 * np.mean(np.any(np.abs(u3) > span, 1))),
                xy_pct=float(100 * np.mean(np.any(np.abs(u2) > span, 1))))


def check_harness() -> float:
    """|RMSE difference| between track() and evaluate_controller on the scored circle."""
    task = CircularTrackingTask()
    name, _, make = baseline_set()[-1]
    ref = evaluate(make(), task, SIM_SEED).rmse_mm
    r = track(make(), lambda home, k: make_reference(home, k / HZ, task), task.track_steps)
    mine = planar_metrics(1000.0 * (r["tips"] - r["refs"]))["planar_rmse_mm"]
    return abs(mine - ref)


# ------------------------------------------------------------------------------ main

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--upstream", type=Path, default=UPSTREAM, help="clone of Jeevan-HM/Soft-Robotic-Arm")
    ap.add_argument("--track-s", type=float, default=180.0)
    args = ap.parse_args()
    n = int(round(args.track_s * HZ))
    t_start = time.perf_counter()

    print(f"harness check (scored circle, {baseline_set()[-1][0]}): "
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
            open_loop_replay=planar_metrics(replays[w][:n] - rec),
            static_infeasible=static_infeasible_pct(disp[:n]),
            controllers={},
        )
        out["tasks"][w] = info
        ref_fn = lambda home, k, d=disp: home + d[k] / 1000.0  # noqa: E731
        for name, _, make in baseline_set():
            t0 = time.perf_counter()
            res = track(make(), ref_fn, n)
            row = planar_metrics(1000.0 * (res["tips"] - res["refs"]), res["cmds"])
            row["run_s"] = time.perf_counter() - t0
            info["controllers"][name] = row
        print(f"  {w}: done ({time.perf_counter() - t_start:.0f} s elapsed)")

    names = [nm for nm, _, _ in baseline_set()]
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
