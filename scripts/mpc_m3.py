"""Delay-compensated MPC on M3 (M2's actuator with second-order mechanics), reference known.

M3 (softarm.models.M3; docs/figures/task_space_model.json M3_mechanics, fitted by
scripts/task_space_model.py), per axis:
    tau p' = -p + G u(t - T),    y'' + 2 zeta w y' + w^2 y = w^2 (H p)
State X = (y, y', p) in R^9: tip (mm), tip velocity (mm/s), measured S2-S4 pressure (psi), all
relative to the settled pose. y' is the backward difference of the measured tip (a signal the
hardware has; the simulator's tip_vel is not used). The controller is softarm.controllers.DelayMPC,
the same law as the MPC on M2: ZOH at 10 ms, prediction across the 50-step dead time with the
commands already sent, offset w (tip prediction error, low-passed 0.5 s), cost
sum |C X(k+N+1+i) + w - r(k+N+i)|^2 + R_u sum |du|^2, closed-form first move, clipped to 0-9 psi.

Horizon 100 steps = 1 s (as the tuned M2 MPC); R_u swept over RU_GRID (that of
compare_m2_controllers.py) on its tuning circle (5 mm, 0.07 Hz, seed 5), never on a test.

Usage (module; run by notebooks/mpc_m3.ipynb):
    uv run python scripts/mpc_m3.py        # tune R_u, print it
"""

import multiprocessing as mp

import numpy as np

from softarm import plotting as P
from softarm.constants import HZ, TRACK_S
from softarm.controllers import DelayMPC
from softarm.evaluation import TESTS, TUNE, circle, ref_array, track_ref_mm
from softarm.labels import RU_GRID, mpc_label
from softarm.models import M3

HORIZON = 100
M3_LABEL = mpc_label("M3", HORIZON)


def _tune_job(r_u: float) -> tuple[float, float]:
    d = ref_array(circle(TUNE["radius_m"], TUNE["frequency_hz"]))
    res = track_ref_mm(DelayMPC(M3.load(), d, HORIZON, r_u), d, int(round(TRACK_S * HZ)), seed=TUNE["seed"])
    e = 1000.0 * np.linalg.norm((res["tips"] - res["refs"])[:, :2], axis=1)
    return r_u, float(np.sqrt(np.mean(e[int(round(HZ / TUNE["frequency_hz"])):] ** 2)))


def tune() -> tuple[float, list]:
    """R_u for the M3 MPC on the tuning circle: (best R_u, [(R_u, RMSE after first lap mm)])."""
    with mp.get_context("fork").Pool(len(RU_GRID)) as pool:
        out = pool.map(_tune_job, RU_GRID)
    return min(out, key=lambda x: x[1])[0], out


def tracking_figure(rows: list, test: str, names: tuple):
    """Tip path (last lap or after 5 s) and planar error over time for each MPC on one test."""
    R = {r["controller"]: r for r in rows if r["test"] == test}
    _, _, settle_s, kind = TESTS[test]
    fig, axs = P.figure(1, 2, layout="constrained")
    axs = np.ravel(axs)
    fig.set_size_inches(P.WIDTH * 1.2, P.PANEL_H * 1.6)
    r0 = R[names[0]]
    t, ref = np.array(r0["t_s"]), np.array(r0["ref_mm"])
    k0 = int(round(settle_s / (t[1] - t[0])))
    sl = slice(k0, None) if kind == "open" else slice(len(t) - k0, None)
    axs[0].plot(ref[sl, 0], ref[sl, 1], **P.REFERENCE)
    for n in names:
        tip = np.array(R[n]["tip_mm"])
        axs[0].plot(tip[sl, 0], tip[sl, 1], **P.line(n, len(tip[sl])))
        e = np.linalg.norm(tip - ref, axis=1)
        axs[1].plot(t, e, **P.line(n, len(t), label=f"{P.label(n)}: {R[n]['metrics']['rmse_settled_mm']:.3f} mm"))
    axs[0].set(xlabel="x − home (mm)", ylabel="y − home (mm)", title=f"tip path, {'after 5 s' if kind == 'open' else 'last lap'}")
    axs[0].set_aspect("equal", adjustable="datalim")
    axs[1].set(xlabel="tracking time (s)", ylabel="planar error (mm)", title="planar error (legend: RMSE after settling)",
               ylim=(0, None))
    for a in axs:
        a.grid(alpha=0.3)
    P.legend_below(axs[0], ncol=1)
    P.legend_below(axs[1], ncol=1)
    fig.suptitle(f"{test} (make_calibrated_sim, seed {TESTS[test][1]})", fontsize=16)
    return fig


if __name__ == "__main__":
    best, grid = tune()
    print("M3 MPC R_u tuning (RMSE after first lap, mm): " + "  ".join(f"{r:g}: {e:.4f}" for r, e in grid))
    print(f"chosen R_u = {best:g}")
