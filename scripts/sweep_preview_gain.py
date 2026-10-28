"""Sweep the reference preview and the sliding gain lambda on the brief's test motions.

Question: is the overshoot of the model-based controllers caused by the gain or by the
reference preview? Each run is one controller configuration on one task, scored exactly as
in scripts/benchmark_slide_tasks.py (same references, same harness loop, same metrics:
planar RMSE after the first lap, shape error, path size %, saturation %).

Configurations:
    delay-aware SMC (M2), Taylor preview  r + T r' + T^2 r''/2,   lambda in LAM_DELAY
    delay-aware SMC (M2), exact preview   r(t + T) passed in,     lambda in LAM_DELAY
    paper SMC (M1, eq. 5), no preview,                            lambda in LAM_PAPER
Other gains are those of scripts/compare_controllers.py (delay-aware: mu = 1 mm, eps = 1,
no observer; paper SMC: eps = 0.1).

Writes docs/figures/sweep_preview_gain.json (every run: task, controller, preview, all
parameters, metrics) and docs/figures/sweep_preview_gain.png.

Usage:
    uv run python scripts/sweep_preview_gain.py [--track-s 30]
"""

import argparse
import json
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import benchmark_all_tasks as B
import benchmark_slide_tasks as S
import ndosmc as N
from soft_robotic_arm.evaluation import CircularTrackingTask, make_reference

LAM_DELAY = (0.5, 1.0, 2.0, 4.0, 8.0)
LAM_PAPER = (0.25, 0.5, 1.0, 2.0)
DELAY_FIXED = dict(mu=1.0, eps=1.0, observer=False)
PAPER_FIXED = dict(eps=0.1)


def references(n: int) -> dict:
    """The five brief tasks of benchmark_slide_tasks.py: name -> (ref_mm (n, 3), lap_s)."""
    tt = np.arange(n) / S.HZ
    m = S.load_m2()
    b = S.reach_bounds(m)
    circ = lambda r: np.array([1000.0 * make_reference(np.zeros(3), t, CircularTrackingTask(  # noqa: E731
        radius_m=r / 1000.0, frequency_hz=S.CIRCLE_F)) for t in tt])
    tri = np.zeros((n, 3))
    tri[:, :2] = S.triangle_xy(tt, S.floor01(b["triangle_edge_mm"]))[0]
    ax, _ = S.open_loop(S.axial_cmd(S.BASE), n)
    co, _ = S.open_loop(S.circle_ol_cmd, n)
    return {
        "axial": (ax, 1 / S.AXIAL_F),
        "circle_ol": (co, 1 / S.CIRCLE_F),
        "circle_static": (circ(S.floor01(b["circle_static_mm"])), 1 / S.CIRCLE_F),
        "circle_dyn": (circ(S.floor01(b["circle_dyn_0p4hz_mm"])), 1 / S.CIRCLE_F),
        "triangle": (tri, S.TRI_PERIOD),
    }


def configurations(m1) -> list[dict]:
    cfg = []
    for lam in LAM_DELAY:
        for preview in ("taylor", "exact"):
            params = dict(lam=lam, **DELAY_FIXED, exact_preview=preview == "exact")
            cfg.append(dict(controller="delay-aware SMC (M2)", preview=preview, params=params,
                            make=lambda ref, p=params: (S.Ahead(N.DelayAwareNDOSMC(m1, **p), ref)
                                                        if p["exact_preview"] else N.DelayAwareNDOSMC(m1, **p))))
    for lam in LAM_PAPER:
        params = dict(lam=lam, **PAPER_FIXED)
        cfg.append(dict(controller="paper SMC (M1)", preview="none", params=params,
                        make=lambda ref, p=params: N.PaperSMC(m1, **p)))
    return cfg


def figure(runs: list[dict], path) -> None:
    fig, axs = plt.subplots(2, 2, figsize=(11, 7), sharex="col")
    styles = {("delay-aware SMC (M2)", "taylor"): ("#6a4c9c", "o-", "delay-aware, Taylor preview"),
              ("delay-aware SMC (M2)", "exact"): ("#c2185b", "s-", "delay-aware, exact r(t+T)"),
              ("paper SMC (M1)", "none"): ("#9a6a12", "^-", "paper SMC")}
    for j, task in enumerate(("circle_dyn", "triangle")):
        for (ctl, pv), (col, mk, lab) in styles.items():
            rs = sorted((r for r in runs if r["task"] == task and r["controller"] == ctl and r["preview"] == pv),
                        key=lambda r: r["params"]["lam"])
            lam = [r["params"]["lam"] for r in rs]
            axs[0, j].plot(lam, [r["metrics"]["planar_rmse_after_lap_mm"] for r in rs], mk, color=col, label=lab)
            axs[1, j].plot(lam, [r["metrics"]["size_pct_after_lap"] for r in rs], mk, color=col, label=lab)
        axs[0, j].set(title={"circle_dyn": "Circle r = 4.3 mm, 0.4 Hz", "triangle": "Triangle R = 6 mm, 7.5 s lap"}[task],
                      ylabel="planar RMSE after first lap (mm)", ylim=(0, None))
        axs[1, j].axhline(100, color="black", lw=0.8, ls="--")
        axs[1, j].set(xlabel="sliding gain λ (1/s)", ylabel="path size (% of reference)", xscale="log")
        ticks = sorted(set(LAM_DELAY) | set(LAM_PAPER))
        for a in axs[:, j]:
            a.grid(alpha=0.3)
            a.set_xticks(ticks, [f"{t:g}" for t in ticks])
            a.minorticks_off()
    axs[0, 0].legend(fontsize=8)
    fig.suptitle("Gain does not remove the overshoot; the exact preview does (make_calibrated_sim seed 0)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--track-s", type=float, default=30.0)
    args = ap.parse_args()
    n = int(round(args.track_s * S.HZ))
    t0 = time.perf_counter()
    m1 = N.TaskSpaceModel.load()
    refs = references(n)
    runs = []
    for task, (ref, lap) in refs.items():
        for c in configurations(m1):
            res = B.track(c["make"](ref), lambda home, k, d=ref: home + d[k] / 1000.0, n)
            met = S.score(1000.0 * (res["tips"] - res["home"]), 1000.0 * (res["refs"] - res["home"]), lap, res["cmds"])
            runs.append(dict(task=task, controller=c["controller"], preview=c["preview"], params=c["params"],
                             sim=f"make_calibrated_sim(control_hz={S.HZ}, seed={S.SEED})", track_s=args.track_s,
                             lap_s=lap, metrics=met))
            r = runs[-1]
            print(f"{task:<14}{c['controller']:<22}{c['preview']:<8}λ={c['params']['lam']:<5g}"
                  f"rmse>lap {met['planar_rmse_after_lap_mm']:6.2f}  shape {met['shape_rms_after_lap_mm']:5.2f}"
                  f"  size {met['size_pct_after_lap']:4.0f}%  sat {met['saturated_pct']:5.1f}%")
    out = dict(description=__doc__.splitlines()[0], lam_delay=LAM_DELAY, lam_paper=LAM_PAPER,
               delay_fixed=DELAY_FIXED, paper_fixed=PAPER_FIXED, runs=runs,
               total_run_s=time.perf_counter() - t0)
    (S.FIG / "sweep_preview_gain.json").write_text(json.dumps(out, indent=2))
    figure(runs, S.FIG / "sweep_preview_gain.png")
    print(f"\n{len(runs)} runs in {out['total_run_s']:.0f} s; wrote docs/figures/sweep_preview_gain.json and .png")


if __name__ == "__main__":
    main()
