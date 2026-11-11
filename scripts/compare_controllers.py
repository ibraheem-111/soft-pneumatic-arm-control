"""Compare every controller built for the calibrated plant, on the package's own harness.

Runs each controller at its chosen gains on
  (1) the scored task: CircularTrackingTask() on make_calibrated_sim(seed=0), and
  (2) a held-out task: 10 mm circle at 0.15 Hz on seed 5,
because the gains were picked by sweeping on the scored task itself.
Prints a table (whole-run RMSE as scored, RMSE after the first 5 s, and the share of
command samples at 0 or 9 psi), and writes
docs/figures/controller_comparison.png and docs/figures/controller_comparison.json.

Controllers: softarm.controllers.presets.baseline_set(): the notebook's PD and SMC (as written
and with the corrected allocator), and the Qiao, Tao & Zhang SMC / NDOSMC and the delay-aware
variants (softarm.controllers). Tasks: softarm.evaluation.motions.CIRCLE_TASKS.

Usage:
    uv run python scripts/compare_controllers.py
"""

import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from soft_robotic_arm.evaluation import TrackingResult

from softarm.constants import FIG
from softarm.controllers.presets import baseline_set
from softarm.evaluation import evaluate
from softarm.evaluation.motions import CIRCLE_TASKS

TASKS = CIRCLE_TASKS


def tail_rmse(r: TrackingResult, after_s: float = 5.0) -> float:
    sel = r.times >= after_s
    err = np.linalg.norm(r.tips[sel, :2] - r.references[sel, :2], axis=1) * 1000.0
    return float(np.sqrt(np.mean(err**2)))


def main() -> None:
    out = {}
    results = {}
    for task_name, (task, seed) in TASKS.items():
        print(f"\n{task_name}")
        print(f"{'controller':<60}{'RMSE':>7}{'t>5s':>7}{'lag s':>7}{'effort':>8}{'sat %':>7}")
        for name, family, make in baseline_set():
            r = evaluate(make(), task, seed)
            results[(task_name, name)] = r
            row = dict(family=family, rmse_mm=r.rmse_mm, rmse_after_5s_mm=tail_rmse(r),
                       max_error_mm=r.max_error_mm, phase_lag_s=r.phase_lag_s, effort_psi=r.effort_psi,
                       saturated_pct=float(100 * np.mean((r.commands <= 0.0) | (r.commands >= 9.0))))
            out.setdefault(task_name, {})[name] = row
            print(f"{name:<60}{r.rmse_mm:>7.2f}{row['rmse_after_5s_mm']:>7.2f}"
                  f"{r.phase_lag_s:>7.2f}{r.effort_psi:>8.2f}{row['saturated_pct']:>7.1f}")
    (FIG / "controller_comparison.json").write_text(json.dumps(out, indent=2))

    # figure: scored task, tip error over time and x-y paths for four representative controllers
    task_name = next(iter(TASKS))
    pick = ["PD, corrected allocator", "SMC (notebook model), corrected allocator + deviation, λ=8",
            "Paper SMC, eq. 5 (M1)", "Delay-aware NDOSMC (M2)"]
    colors = ["#8a8f93", "#1f6f7a", "#9a6a12", "#6a4c9c"]
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6), gridspec_kw=dict(width_ratios=[1.7, 1]))
    for name, col in zip(pick, colors):
        r = results[(task_name, name)]
        err = np.linalg.norm(r.tips[:, :2] - r.references[:, :2], axis=1) * 1000.0
        axes[0].plot(r.times, err, color=col, lw=1.3, label=f"{name} ({r.rmse_mm:.2f} mm)")
        axes[1].plot((r.tips[:, 0] - r.home[0]) * 1000, (r.tips[:, 1] - r.home[1]) * 1000, color=col, lw=1.1)
    r0 = results[(task_name, pick[0])]
    axes[1].plot((r0.references[:, 0] - r0.home[0]) * 1000, (r0.references[:, 1] - r0.home[1]) * 1000,
                 "--", color="black", lw=1, label="reference")
    axes[0].set(xlabel="tracking time (s)", ylabel="lateral error (mm)", ylim=(0, 8),
                title="Scored task: lateral tracking error")
    axes[0].legend(fontsize=8.5, loc="upper right")
    axes[0].grid(alpha=0.3)
    axes[1].set(xlabel="x − home (mm)", ylabel="y − home (mm)", title="Tip path", aspect="equal")
    axes[1].legend(fontsize=8.5)
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG / "controller_comparison.png", dpi=140)
    print(f"\nwrote {FIG / 'controller_comparison.png'} and controller_comparison.json")


if __name__ == "__main__":
    main()
