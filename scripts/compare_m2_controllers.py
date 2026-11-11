"""Compare controllers built on the identified M2 model, with the reference known in advance.

The professor confirmed the reference trajectory may be assumed known (an arm is given its
reference), so every controller here reads the future reference instead of estimating it.

Controllers (all on M2: docs/figures/task_space_model.json):
  Delay-aware SMC (M2, known ref)     softarm.controllers.DelayAwareNDOSMC, exact_preview=True,
                                      lam=8, mu=1 mm, eta=1 mm/s, no observer (gains of
                                      compare_controllers.py)
  Delay-aware NDOSMC (M2, known ref)  same with the observer, Nikoletta's notebook gains
                                      (lam=4, l1=0.1, mu=1 mm, Phi=1, eps=1)
  Delay-aware SMC (M2, known ref),    the SMC at the NDOSMC's lam=4 without the observer, so the
  lambda = 4                          observer's effect is separated from the gain change
  MPC (M2), horizon H steps           softarm.controllers.DelayMPC on M2, H in HORIZONS
                                      (H steps x 10 ms after the 0.5 s dead time)
Also run, for comparison:
  MPC (F), horizon 300 steps          softarm.controllers.KalmanDelayMPC (known_reference) on the
                                      frequency-identified model F (models/sysid_calibrated_v2.json),
                                      its own tuned rho = 1e-5 m^2/psi^2, given the true future
                                      reference (preview="oracle")

DelayMPC on M2, every 10 ms step k (state X = (y, p): tip mm and measured S2-S4 pressure psi, both as
deviations from the settled pose; input u = command - 4.5 psi):
  1. X_d = (A_d, B_d): M2 discretised with zero-order hold; u acts after N = 50 steps.
  2. Predict across the dead time from the measured state with the commands already sent:
     X(k+N) = A^N X(k) + sum_i A^(N-1-i) B u(k-N+i).
  3. Model-error correction w: tip measured now minus the tip predicted for now N steps ago,
     low-pass filtered (time constant 0.5 s), held constant over the horizon.
  4. Minimise sum_{i=0..H-1} |C X(k+N+1+i) + w - r(k+N+i)|^2 + R_u sum |u(k+i) - u(k+i-1)|^2
     over u(k..k+H-1) (unconstrained, closed form, first move applied, then clipped to
     0-9 psi; the clipped command enters step 2). r: the known reference. Output k+N+1+i is
     paired with r(k+N+i) because the harness scores the tip after step j against r(j).
  R_u per horizon: swept on a tuning circle (5 mm, 0.07 Hz, seed 5), never on a test.
  With a small R_u the first move is close to an inverse of the model and the horizon barely
  matters; the horizon matters when command changes are penalised, so the script also runs
  every horizon at the fixed weights RU_FIXED ("horizon sensitivity" in the JSON).

Tests (make_calibrated_sim(control_hz=100), harness loop softarm.evaluation.track; tests softarm.evaluation.TESTS):
  held-out circle   10 mm, 0.15 Hz, seed 5 (compare_controllers.py), 30 s
  scored circle     7 mm, 0.1 Hz, seed 0 (package CircularTrackingTask), 30 s
  squiggle          smooth random: per axis 6 sines, random 0.03-0.3 Hz, phases, weights,
                    +-6 mm, seed 101, sim seed 0, 30 s (as notebooks/Soft_robotic_arm_SMC_M2)
  random waypoints  straight lines between random points (+-6 mm, 1-2.5 s), seed 101
  triangle          vertices on a 7 mm circle (first at +x), 10 s per lap, seed 0
                    (as scripts/motion_tests.py)
  square            axis-aligned, vertices on a 7 mm circle (side 9.9 mm), 10 s per lap, seed 0
Metrics (planar x-y, softarm.evaluation.score): RMSE whole run; RMSE after settling (first lap for circles, 5 s for
the random paths); shape error = RMS distance from the tip to the nearest point of the
reference path after settling (timing ignored); max error after settling; radius range
(circles); path size % (polygons: RMS distance from the centre of the traced path over that of
the reference, 100 = right size); % of commands at 0 or 9 psi; y phase lag.

Writes docs/figures/m2_controllers.json (every run: controller, all gains, test, metrics, and
the tip/reference x-y traces every 50 ms) and docs/figures/m2ctrl_<test>.png.

The notebook notebooks/m2_controller_comparison.ipynb runs the same comparison and shows the
tables and figures inline; this module holds the code it calls.

Usage:
    uv run python scripts/compare_m2_controllers.py            # tune R_u, run, write (about 30 s)
"""

import json
import multiprocessing as mp
import os
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import numpy as np  # noqa: E402

from softarm import plotting as P  # noqa: E402
from softarm.constants import DT, F_MODEL_JSON, FIG, HZ, TRACK_S  # noqa: E402
from softarm.controllers import Ahead, DelayAwareNDOSMC, DelayMPC, KalmanDelayMPC, known_reference  # noqa: E402
from softarm.controllers.kalman_mpc import HORIZON as F_HORIZON, RHO as F_RHO  # noqa: E402
from softarm.evaluation import TESTS, TUNE, circle, ref_array, score, track_ref_mm  # noqa: E402
from softarm.labels import MPC_F, NDOSMC_KNOWN, RU_GRID, SMC_KNOWN, SMC_L4, mpc_label as _mpc_label  # noqa: E402
from softarm.models import M2  # noqa: E402

HORIZONS = (25, 100, 300)                   # MPC horizons, steps after the dead time
RU_FIXED = (1.0, 10.0)                      # horizon sensitivity at a fixed, larger R_u
TRACE_EVERY = 5                             # traces stored in the JSON every 50 ms (metrics use 100 Hz)


# ------------------------------------------------------------------------- controllers

def controllers(r_u: dict) -> dict:
    m = M2.load()
    out = {
        SMC_KNOWN: lambda d: Ahead(DelayAwareNDOSMC(
            m, lam=8.0, mu=1.0, eps=1.0, observer=False, exact_preview=True), d),
        NDOSMC_KNOWN: lambda d: Ahead(DelayAwareNDOSMC(
            m, lam=4.0, l1=0.1, l2=0.0, mu=1.0, Phi=1.0, eps=1.0, exact_preview=True), d),
        SMC_L4: lambda d: Ahead(DelayAwareNDOSMC(
            m, lam=4.0, mu=1.0, eps=1.0, observer=False, exact_preview=True), d),
        MPC_F: known_reference,
    }
    for h in HORIZONS:
        out[mpc_label(h)] = lambda d, h=h: DelayMPC(m, d, h, r_u[h])
    for ru in RU_FIXED:
        for h in HORIZONS:
            out[mpc_label(h, ru)] = lambda d, h=h, ru=ru: DelayMPC(m, d, h, ru)
    return out


def mpc_label(h: int, r_u: float | None = None) -> str:
    """Label of the M2 MPC at horizon h (and a fixed R_u)."""
    return _mpc_label("M2", h, r_u)


def gains_of(ctl) -> dict:
    if isinstance(ctl, DelayMPC):
        return ctl.gains
    if isinstance(ctl, KalmanDelayMPC):
        return dict(horizon_steps=F_HORIZON, horizon_s=F_HORIZON * DT, rho_m2_per_psi2=F_RHO, preview="oracle",
                    dead_time_steps=ctl.axes[0].d, model=F_MODEL_JSON.name)
    return (ctl.c if hasattr(ctl, "c") else ctl).gains


# ----------------------------------------------------------------------------- runs

def run(job):
    test, name, r_u = job
    fn, seed, settle_s, kind = TESTS[test]
    d = ref_array(fn)
    ctl = controllers(r_u)[name](d)
    t0 = time.perf_counter()
    res = track_ref_mm(ctl, d, int(round(TRACK_S * HZ)), seed=seed)
    tip, ref = 1000.0 * (res["tips"] - res["home"]), 1000.0 * (res["refs"] - res["home"])
    return dict(test=test, controller=name, seed=seed, gains=gains_of(ctl), run_s=time.perf_counter() - t0,
                metrics=score(tip, ref, settle_s, res["cmds"], kind),
                tip_mm=tip[:, :2].round(4).tolist(), ref_mm=ref[:, :2].round(4).tolist())


def tune_job(job):
    h, r_u = job
    fn = circle(TUNE["radius_m"], TUNE["frequency_hz"])
    d = ref_array(fn)
    res = track_ref_mm(DelayMPC(M2.load(), d, h, r_u), d, int(round(TRACK_S * HZ)), seed=TUNE["seed"])
    e = 1000.0 * np.linalg.norm((res["tips"] - res["refs"])[:, :2], axis=1)
    k0 = int(round(HZ / TUNE["frequency_hz"]))
    return h, r_u, float(np.sqrt(np.mean(e[k0:] ** 2)))


# ----------------------------------------------------------------------------- figures

PANELS = [  # never blue next to reddish purple, or green next to orange, in one panel
    ("SMC vs MPC, known reference",
     ["Delay-aware SMC (M2, known ref)", "Delay-aware NDOSMC (M2, known ref)", mpc_label(100), MPC_F]),
    ("MPC (M2) horizons at R_u = 10", [mpc_label(h, 10.0) for h in HORIZONS]),
]


def figure(rows: list, test: str, path: Path | None = None):
    """Tip paths and error over time for one test; saved to path if given, returned otherwise."""
    R = {r["controller"]: r for r in rows if r["test"] == test}
    _, _, settle_s, kind = TESTS[test]
    k0 = int(round(settle_s * HZ))
    fig, axs = P.figure(len(PANELS), 2, layout="constrained")
    fig.set_size_inches(P.WIDTH * 1.2, P.PANEL_H * len(PANELS) * 1.35)
    for row, (title, names) in enumerate(PANELS):
        ax = axs[row, 0]
        ref = np.array(next(iter(R.values()))["ref_mm"])
        sl = slice(k0, None) if kind == "open" else slice(len(ref) - k0, None)
        ax.plot(ref[sl, 0], ref[sl, 1], **P.REFERENCE)
        for nm in names:
            tip = np.array(R[nm]["tip_mm"])
            ax.plot(tip[sl, 0], tip[sl, 1], **P.line(nm, len(tip[sl])))
        ax.set(xlabel="x − home (mm)", ylabel="y − home (mm)",
               title=f"{title}: {'after 5 s' if kind == 'open' else 'last lap'}")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
        ax.legend(["reference"] + [P.label(n) for n in names], loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=1)
        ax = axs[row, 1]
        t = np.arange(len(ref)) * DT
        for nm in names:
            e = np.linalg.norm(np.array(R[nm]["tip_mm"]) - ref, axis=1)
            ax.plot(t, e, **P.line(nm, len(t), label=f"{P.label(nm)}: {R[nm]['metrics']['rmse_settled_mm']:.2f} mm"))
        ax.set(xlabel="tracking time (s)", ylabel="planar error (mm)", title="Planar error over time (legend: RMSE after settling)")
        ax.set_ylim(0, None)
        ax.grid(alpha=0.3)
        P.legend_below(ax, ncol=1)
    fig.suptitle(f"{test} (make_calibrated_sim, seed {TESTS[test][1]})", fontsize=16)
    if path is None:
        return fig
    P.save(fig, path)


# -------------------------------------------------------------------------------- main

def tune(workers: int = 10) -> tuple[dict, list]:
    """Sweep R_u per horizon on the tuning circle; return ({horizon: best R_u}, all results)."""
    with mp.get_context("fork").Pool(workers) as pool:
        tuned = pool.map(tune_job, [(h, r) for h in HORIZONS for r in RU_GRID])
    r_u = {h: min(((r, e) for hh, r, e in tuned if hh == h), key=lambda x: x[1])[0] for h in HORIZONS}
    return r_u, tuned


def run_all(r_u: dict, workers: int = 10) -> list:
    """Every controller on every test; one dict per run (see run())."""
    names = list(controllers(r_u))
    with mp.get_context("fork").Pool(workers) as pool:
        return pool.map(run, [(t, n, r_u) for t in TESTS for n in names])


def save(rows: list, r_u: dict, tuned: list, total_s: float) -> None:
    out = dict(description=__doc__.split("\n\n")[0], dt_s=DT, track_s=TRACK_S, horizons_steps=HORIZONS,
               r_u_tuning=[dict(horizon_steps=h, r_u=r, rmse_after_lap_mm=e) for h, r, e in tuned], r_u=r_u,
               tune_task=TUNE, trace_step_s=TRACE_EVERY * DT,
               runs=[{**r, "tip_mm": r["tip_mm"][::TRACE_EVERY], "ref_mm": r["ref_mm"][::TRACE_EVERY]} for r in rows],
               total_run_s=total_s)
    (FIG / "m2_controllers.json").write_text(json.dumps(out))


def main() -> None:
    t0 = time.perf_counter()
    P.apply()
    r_u, tuned = tune()
    print("R_u tuning (tuning circle 5 mm, 0.07 Hz, seed 5; RMSE after the first lap, mm):")
    for h in HORIZONS:
        row = [(r, e) for hh, r, e in tuned if hh == h]
        print(f"  {mpc_label(h):<40}" + "  ".join(f"{r:g}: {e:.3f}" for r, e in row) + f"   -> R_u = {r_u[h]:g}")
    rows = run_all(r_u)
    print()
    for t in TESTS:
        print(f"{t}:  RMSE whole / RMSE settled / shape / max settled / sat % / lag s / radius or size")
        for r in (r for r in rows if r["test"] == t and "R_u =" not in r["controller"]):
            m = r["metrics"]
            rad = (f"  {m['radius_min_mm']:.1f}-{m['radius_max_mm']:.1f}" if "radius_min_mm" in m
                   else f"  {m['size_pct']:.0f}%" if "size_pct" in m else "")
            print(f"  {r['controller']:<40}{m['rmse_whole_mm']:7.2f}{m['rmse_settled_mm']:8.3f}{m['shape_mm']:8.3f}"
                  f"{m['max_settled_mm']:8.2f}{m['saturated_pct']:7.1f}{m['lag_s']:+7.2f}{rad}")
        figure(rows, t, FIG / f"m2ctrl_{t.replace(' ', '_')}.png")
    print("\nhorizon sensitivity at fixed R_u (RMSE after settling, mm):")
    for ru in RU_FIXED:
        for h in HORIZONS:
            nm = mpc_label(h, ru)
            print(f"  {nm:<52}" + "  ".join(f"{t}: {r['metrics']['rmse_settled_mm']:.3f}"
                                             for t in TESTS for r in rows if r['test'] == t and r['controller'] == nm))
    save(rows, r_u, tuned, time.perf_counter() - t0)
    print(f"\ntotal {time.perf_counter() - t0:.0f} s; wrote docs/figures/m2_controllers.json and m2ctrl_<test>.png")


if __name__ == "__main__":
    main()
