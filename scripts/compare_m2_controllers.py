"""Compare controllers built on the identified M2 model, with the reference known in advance.

The professor confirmed the reference trajectory may be assumed known (an arm is given its
reference), so every controller here reads the future reference instead of estimating it.

Controllers (all on M2: docs/figures/task_space_model.json):
  Delay-aware SMC (M2, known ref)     scripts/ndosmc.py DelayAwareNDOSMC, exact_preview=True,
                                      lam=8, mu=1 mm, eta=1 mm/s, no observer (gains of
                                      compare_controllers.py)
  Delay-aware NDOSMC (M2, known ref)  same with the observer, Nikoletta's notebook gains
                                      (lam=4, l1=0.1, mu=1 mm, Phi=1, eps=1)
  Delay-aware SMC (M2, known ref),    the SMC at the NDOSMC's lam=4 without the observer, so the
  lambda = 4                          observer's effect is separated from the gain change
  MPC (M2), horizon H steps           M2MPC below, H in HORIZONS (H steps x 10 ms after the
                                      0.5 s dead time)
Also run, for comparison:
  MPC (F), horizon 300 steps          scripts/mpc_controller.py DelayMPCController on the
                                      frequency-identified model F (models/sysid_calibrated_v2.json),
                                      its own tuned rho = 1e-5 m^2/psi^2, given the true future
                                      reference (preview="oracle")

M2MPC, every 10 ms step k (state X = (y, p): tip mm and measured S2-S4 pressure psi, both as
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

Tests (make_calibrated_sim(control_hz=100), harness loop benchmark_all_tasks.track):
  held-out circle   10 mm, 0.15 Hz, seed 5 (compare_controllers.py), 30 s
  scored circle     7 mm, 0.1 Hz, seed 0 (package CircularTrackingTask), 30 s
  squiggle          smooth random: per axis 6 sines, random 0.03-0.3 Hz, phases, weights,
                    +-6 mm, seed 101, sim seed 0, 30 s (as notebooks/Soft_robotic_arm_SMC_M2)
  random waypoints  straight lines between random points (+-6 mm, 1-2.5 s), seed 101
  triangle          vertices on a 7 mm circle (first at +x), 10 s per lap, seed 0
                    (as scripts/motion_tests.py)
  square            axis-aligned, vertices on a 7 mm circle (side 9.9 mm), 10 s per lap, seed 0
Metrics (planar x-y): RMSE whole run; RMSE after settling (first lap for circles, 5 s for
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
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.linalg import expm  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import benchmark_all_tasks as B  # noqa: E402
import benchmark_slide_tasks as BS  # noqa: E402
import ndosmc as N  # noqa: E402
import mpc_controller as F  # noqa: E402
import plot_style as P  # noqa: E402
from soft_robotic_arm.evaluation import CircularTrackingTask, _phase_lag_seconds, make_reference  # noqa: E402

HZ = 100.0
DT = 1.0 / HZ
TRACK_S = 30.0
PAD = 400                                   # reference samples beyond the run (look-ahead)
HORIZONS = (25, 100, 300)                   # MPC horizons, steps after the dead time
RU_GRID = (1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0)     # mm^2 / psi^2
RU_FIXED = (1.0, 10.0)                      # horizon sensitivity at a fixed, larger R_u
TUNE = dict(radius_m=0.005, frequency_hz=0.07, seed=5)
FIG = B.FIG
TRACE_EVERY = 5                             # traces stored in the JSON every 50 ms (metrics use 100 Hz)


# ------------------------------------------------------------------------------- MPC

class M2MPC:
    """Delay-compensated, unconstrained MPC on M2 with the reference known in advance."""

    def __init__(self, ref_mm: np.ndarray, horizon: int, r_u: float, model: N.TaskSpaceModel | None = None,
                 offset_tau_s: float = 0.5):
        m = model or N.TaskSpaceModel.load()
        act = m.actuator
        lam = np.diag([ax.params(0.0)[0] for ax in m.axes_measured])
        H = m.B_measured
        Ac = np.block([[-lam, lam @ H], [np.zeros((3, 3)), -np.eye(3) / act.tau_s]])
        Bc = np.vstack([np.zeros((3, 3)), np.diag(act.gain) / act.tau_s])
        E = expm(np.block([[Ac, Bc], [np.zeros((3, 9))]]) * DT)
        self.A, self.B = E[:6, :6], E[:6, 6:]
        self.C = np.hstack([np.eye(3), np.zeros((3, 3))])
        self.nd = int(round(act.delay_s * HZ))
        self.h, self.r_u, self.d = horizon, r_u, ref_mm
        self.alpha = 1.0 - np.exp(-DT / offset_tau_s)
        self.gains = dict(horizon_steps=horizon, horizon_s=horizon * DT, r_u=r_u, dead_time_steps=self.nd,
                          offset_tau_s=offset_tau_s, model="M2 (task_space_model.json)")
        # Prediction matrices over the horizon: y(k+N+1+i) = Phi_i X(k+N) + sum_j Gam_ij u(k+j)
        n, h = 6, horizon
        Apow = [np.eye(n)]
        for _ in range(max(h, self.nd) + 1):
            Apow.append(self.A @ Apow[-1])
        self.Apow = Apow
        Phi = np.vstack([self.C @ Apow[i + 1] for i in range(h)])                  # (3h, 6)
        Gam = np.zeros((3 * h, 3 * h))
        for i in range(h):
            for j in range(i + 1):
                Gam[3 * i:3 * i + 3, 3 * j:3 * j + 3] = self.C @ Apow[i - j] @ self.B
        D = np.eye(3 * h) - np.eye(3 * h, k=-3)
        Mi = np.linalg.inv(Gam.T @ Gam + r_u * D.T @ D)
        K = Mi[:3]                                                               # first move only
        self.g_r = K @ Gam.T                                                     # (3, 3h)
        self.g_x = -self.g_r @ Phi                                               # (3, 6)
        self.g_w = -self.g_r @ np.tile(np.eye(3), (h, 1))                         # (3, 3)
        E1 = np.zeros((3 * h, 3)); E1[:3] = np.eye(3)
        self.g_u = r_u * K @ D.T @ E1                                            # (3, 3)
        self.W = [Apow[self.nd - 1 - i] @ self.B for i in range(self.nd)]       # delay predictor
        self.k = 0
        self.home = self.p0 = None
        self.buf = [np.zeros(3)] * self.nd
        self.u_prev = np.zeros(3)
        self.w = np.zeros(3)
        self.pred_hist: list = []

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        if self.home is None:
            self.home = np.asarray(obs["tip_pos"], float).copy()
            self.p0 = np.asarray(obs["segment_pressures"], float)[1:4].copy()
        y = 1000.0 * (np.asarray(obs["tip_pos"], float) - self.home)
        p = np.asarray(obs["segment_pressures"], float)[1:4] - self.p0
        X = np.concatenate([y, p])
        Xf = self.Apow[self.nd] @ X + sum(Wi @ ui for Wi, ui in zip(self.W, self.buf))
        self.pred_hist.append(self.C @ Xf)                     # predicted tip at k + N
        if len(self.pred_hist) > self.nd:
            self.w += self.alpha * ((y - self.pred_hist.pop(0)) - self.w)
        k, h, d = self.k, self.h, self.d
        r = d[min(k + self.nd, len(d) - h):][:h]                # r(k+N .. k+N+h-1), mm from home
        r = np.vstack([r, np.repeat(r[-1:], h - len(r), axis=0)]) if len(r) < h else r
        u = self.g_r @ r.reshape(-1) + self.g_x @ Xf + self.g_w @ self.w + self.g_u @ self.u_prev
        cmd = np.clip(4.5 + u, 0.0, 9.0)
        u_app = cmd - 4.5
        self.buf = self.buf[1:] + [u_app]
        self.u_prev = u_app
        self.k += 1
        return cmd


# ---------------------------------------------------------------------------- tests

def smooth_random(seed: int, span_mm: float = 6.0, f_lo: float = 0.03, f_hi: float = 0.3, n: int = 6):
    """Offset from home (m) vs time: per axis a sum of n random sines, scaled to +-span_mm
    (same as notebooks/Soft_robotic_arm_SMC_M2.ipynb)."""
    g = np.random.default_rng(seed)
    f = g.uniform(f_lo, f_hi, (2, n)); ph = g.uniform(0, 2 * np.pi, (2, n)); a = g.uniform(0.3, 1.0, (2, n))
    raw = lambda t: (a * np.sin(2 * np.pi * f * np.asarray(t)[..., None, None] + ph)).sum(-1)  # noqa: E731
    tt = np.arange(0.0, TRACK_S, DT)
    scale = span_mm / 1000.0 / np.abs(raw(tt) - raw(0.0)).max(axis=0)
    return lambda tau: np.append((raw(tau) - raw(0.0)) * scale, 0.0)


def random_waypoints(seed: int, span_mm: float = 6.0, seg_lo: float = 1.0, seg_hi: float = 2.5):
    """Offset from home (m): straight lines between random waypoints (as the notebook)."""
    g = np.random.default_rng(seed)
    times, pts = [0.0], [np.zeros(2)]
    while times[-1] < TRACK_S + PAD * DT + 1.0:
        times.append(times[-1] + g.uniform(seg_lo, seg_hi))
        pts.append(g.uniform(-span_mm, span_mm, 2) / 1000.0)
    pts = np.array(pts)
    return lambda tau: np.array([np.interp(tau, times, pts[:, 0]), np.interp(tau, times, pts[:, 1]), 0.0])


def polygon(n: int, radius_m: float, lap_s: float, first_deg: float = 0.0):
    """Offset from home (m): regular n-gon with vertices on radius_m, one lap per lap_s."""
    th = np.radians(first_deg) + 2 * np.pi * np.arange(n + 1) / n
    v = radius_m * np.column_stack([np.cos(th), np.sin(th)])
    def fn(tau):
        s = (tau / lap_s % 1.0) * n
        k = int(s)
        return np.append(v[k] + (s - k) * (v[k + 1] - v[k]), 0.0)
    return fn


def circle(radius_m: float, freq: float):
    task = CircularTrackingTask(radius_m=radius_m, frequency_hz=freq)
    return lambda tau: make_reference(np.zeros(3), tau, task)


TESTS = {  # name: (offset_fn(tau) -> m, sim seed, settle_s for "after settling", kind)
    "held-out circle": (circle(0.010, 0.15), 5, 1 / 0.15, "circle"),
    "scored circle": (circle(0.007, 0.10), 0, 10.0, "circle"),
    "triangle": (polygon(3, 0.007, 10.0), 0, 10.0, "polygon"),
    "square": (polygon(4, 0.007, 10.0, 45.0), 0, 10.0, "polygon"),
    "squiggle": (smooth_random(101), 0, 5.0, "open"),
    "random waypoints": (random_waypoints(101), 0, 5.0, "open"),
}


def ref_array(fn) -> np.ndarray:
    n = int(round(TRACK_S * HZ)) + PAD
    return np.array([1000.0 * fn(k * DT) for k in range(n)])


# ------------------------------------------------------------------------- controllers

def controllers(r_u: dict) -> dict:
    m = N.TaskSpaceModel.load()
    out = {
        "Delay-aware SMC (M2, known ref)": lambda d: BS.Ahead(N.DelayAwareNDOSMC(
            m, lam=8.0, mu=1.0, eps=1.0, observer=False, exact_preview=True), d),
        "Delay-aware NDOSMC (M2, known ref)": lambda d: BS.Ahead(N.DelayAwareNDOSMC(
            m, lam=4.0, l1=0.1, l2=0.0, mu=1.0, Phi=1.0, eps=1.0, exact_preview=True), d),
        SMC_L4: lambda d: BS.Ahead(N.DelayAwareNDOSMC(
            m, lam=4.0, mu=1.0, eps=1.0, observer=False, exact_preview=True), d),
        MPC_F: mpc_f,
    }
    for h in HORIZONS:
        out[mpc_label(h)] = lambda d, h=h: M2MPC(d, h, r_u[h], m)
    for ru in RU_FIXED:
        for h in HORIZONS:
            out[mpc_label(h, ru)] = lambda d, h=h, ru=ru: M2MPC(d, h, ru, m)
    return out


SMC_L4 = "Delay-aware SMC (M2, known ref), λ = 4"         # NDOSMC's gain without the observer
MPC_F = f"MPC (F), horizon {F.HORIZON} steps = {F.HORIZON * DT:.2f} s"


def mpc_f(ref_mm: np.ndarray) -> F.DelayMPCController:
    """MPC on F with the true future reference; tip0 (the settled tip) is set on its first call."""
    ctl = F.DelayMPCController(preview="oracle")
    ctl.oracle = lambda n: ctl.tip0 + ref_mm[n, :2] / 1000.0
    return ctl


def mpc_label(h: int, r_u: float | None = None) -> str:
    return f"MPC (M2), horizon {h} steps = {h * DT:.2f} s" + ("" if r_u is None else f", R_u = {r_u:g}")


def gains_of(ctl) -> dict:
    if isinstance(ctl, M2MPC):
        return ctl.gains
    if isinstance(ctl, F.DelayMPCController):
        return dict(horizon_steps=F.HORIZON, horizon_s=F.HORIZON * DT, rho_m2_per_psi2=F.RHO, preview="oracle",
                    dead_time_steps=ctl.axes[0].d, model=F.MODEL_PATH.name)
    c = ctl.c if hasattr(ctl, "c") else ctl
    return dict(lam=c.lam, mu=c.mu, eps=c.eps, observer=c.use_observer, Phi=c.Phi,
                l1=c.obs.l1, l2=c.obs.l2, exact_preview=c.exact_preview, dead_time_steps=c.n_delay)


# ----------------------------------------------------------------------------- scoring

def score(tip: np.ndarray, ref: np.ndarray, settle_s: float, cmds: np.ndarray, kind: str) -> dict:
    e = np.linalg.norm(tip[:, :2] - ref[:, :2], axis=1)
    k0 = int(round(settle_s * HZ))
    path = ref[k0:, :2] if kind == "open" else ref[-k0:, :2]      # periodic: one lap of the path
    dense = np.vstack([np.linspace(a, b, 5, endpoint=False) for a, b in zip(path[:-1], path[1:])])
    t = tip[k0:, :2]
    near = np.concatenate([np.sqrt(((t[i:i + 300, None] - dense[None]) ** 2).sum(-1)).min(1)
                           for i in range(0, len(t), 300)])
    out = dict(rmse_whole_mm=float(np.sqrt(np.mean(e ** 2))), rmse_settled_mm=float(np.sqrt(np.mean(e[k0:] ** 2))),
               shape_mm=float(np.sqrt(np.mean(near ** 2))), max_settled_mm=float(e[k0:].max()),
               saturated_pct=float(100 * np.mean((cmds <= 0.0) | (cmds >= 9.0))),
               lag_s=float(_phase_lag_seconds(ref[:, 1], tip[:, 1], HZ, max_lag_s=1.0)), settle_s=settle_s)
    if kind == "polygon":
        rms = lambda a: np.sqrt(np.mean(np.sum((a - a.mean(0)) ** 2, 1)))  # noqa: E731
        out.update(size_pct=float(100 * rms(t) / rms(ref[k0:, :2])))
    if kind == "circle":
        rad = np.linalg.norm(t - t.mean(0), axis=1)
        out.update(radius_min_mm=float(rad.min()), radius_max_mm=float(rad.max()),
                   radius_ref_mm=float(np.linalg.norm(ref[0, :2] - ref[:, :2].mean(0))))
    return out


def run(job):
    test, name, r_u = job
    fn, seed, settle_s, kind = TESTS[test]
    d = ref_array(fn)
    ctl = controllers(r_u)[name](d)
    t0 = time.perf_counter()
    res = B.track(ctl, lambda home, k: home + d[k] / 1000.0, int(round(TRACK_S * HZ)), seed=seed)
    tip, ref = 1000.0 * (res["tips"] - res["home"]), 1000.0 * (res["refs"] - res["home"])
    return dict(test=test, controller=name, seed=seed, gains=gains_of(ctl), run_s=time.perf_counter() - t0,
                metrics=score(tip, ref, settle_s, res["cmds"], kind),
                tip_mm=tip[:, :2].round(4).tolist(), ref_mm=ref[:, :2].round(4).tolist())


def tune_job(job):
    h, r_u = job
    fn = circle(TUNE["radius_m"], TUNE["frequency_hz"])
    d = ref_array(fn)
    res = B.track(M2MPC(d, h, r_u), lambda home, k: home + d[k] / 1000.0, int(round(TRACK_S * HZ)), seed=TUNE["seed"])
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
