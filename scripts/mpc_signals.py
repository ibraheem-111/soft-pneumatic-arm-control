"""Pressures, disturbance and error-space phase portraits for the two MPCs on the M2 comparison tests.

The counterpart of smc_signals.py. Runs the MPC on M2 (softarm.controllers.DelayMPC on M2, tuned
horizon 100 steps = 1 s) and the MPC on F (softarm.controllers.KalmanDelayMPC, horizon 300 steps
= 3 s), both given the known reference, on the six tests, and records every 10 ms:
  u        commanded segment pressure S2-S4 (psi)
  p        measured segment pressure S2-S4 (psi)
  d        the disturbance M2 sees, d = x_dot - (-a x + b H (p - p0)) (mm/s), as in smc_signals.py
  offset   the MPC's own disturbance estimate (mm): M2 MPC w (low-passed tip prediction error,
           0.5 s), F MPC o (Kalman-filtered tip offset); a constant output disturbance on the tip
  E        the error the MPC aims to remove first: predicted tip at arrival, plus offset, minus the
           reference then, E = C X(k+N) + w - r(k+N) (mm); E_dot by central differences (mm/s)

Writes docs/figures/mpc_signals.json (traces every 50 ms), docs/figures/mpc_signals_<test>.png and
docs/figures/mpc_phase_<test>.png.

The functions also take the MPC on M3 (DelayMPC on M3, see scripts/mpc_m3.py); notebooks/mpc_m3.ipynb
runs all three.

Usage:
    uv run python scripts/mpc_signals.py
"""

import json
import multiprocessing as mp
import os
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import numpy as np  # noqa: E402

from softarm import plotting as P  # noqa: E402
from softarm.constants import DT, FIG, HZ, TRACK_S  # noqa: E402
from softarm.controllers import DelayMPC, known_reference  # noqa: E402
from softarm.controllers.kalman_mpc import HORIZON as F_HORIZON, RHO as F_RHO  # noqa: E402
from softarm.evaluation import TESTS, ref_array, score, track_ref_mm  # noqa: E402
from softarm.labels import MPC_F, mpc_label  # noqa: E402
from softarm.models import M2 as M2Model, M3 as M3Model  # noqa: E402

M3_HORIZON = 100        # scripts/mpc_m3.py HORIZON
M2, F, M3 = mpc_label("M2", 100), MPC_F, mpc_label("M3", M3_HORIZON)
M3_RU = 1e-5            # mpc_m3.tune() on the tuning circle (5 mm, 0.07 Hz, seed 5)
EVERY = 5


def r_u_tuned() -> float:
    return float(json.loads((FIG / "m2_controllers.json").read_text())["r_u"]["100"])


class Recorder:
    """Wraps either MPC; logs measured pressure, tip, the offset estimate and the predicted error."""

    def __init__(self, name: str, ref_mm: np.ndarray):
        self.name, self.d = name, ref_mm
        self.ctl = (DelayMPC(M2Model.load(), ref_mm, 100, r_u_tuned()) if name == M2
                    else DelayMPC(M3Model.load(), ref_mm, M3_HORIZON, M3_RU) if name == M3 else known_reference(ref_mm))
        self.p, self.x, self.v, self.off, self.E = [], [], [], [], []
        if name == F:
            self.f_pred = [0.0, 0.0]
            for j, ax in enumerate(self.ctl.axes):
                ax.step = self._wrap(ax, j, ax.step)

    def _wrap(self, ax, j, step):
        Cr = ax.H[0][:ax.n]                                       # output row: H = [[C, 1], [e1, 0]]

        def wrapped(tip, pressure, r_future):
            u = step(tip, pressure, r_future)
            x, o = ax.z[:ax.n], ax.z[ax.n]                         # after the measurement update
            x_d = ax.Ad_d @ x + (ax.W @ np.array(ax.buf) if ax.d else 0.0)
            self.f_pred[j] = (float(Cr @ x_d + o), float(o), float(r_future[0]))
            return u
        return wrapped

    def compute(self, t, obs, ref):
        cmd = self.ctl.compute(t, obs, ref)
        self.p.append(np.asarray(obs["segment_pressures"], float)[1:4])
        self.x.append(np.asarray(obs["tip_pos"], float))
        self.v.append(np.asarray(obs["tip_vel"], float))
        c = self.ctl
        if self.name in (M2, M3):
            k = c.k - 1                                            # compute() has already advanced k
            self.off.append(c.w[:2].copy())
            self.E.append(c.pred_hist[-1][:2] + c.w[:2] - self.d[min(k + c.nd, len(self.d) - 1), :2])
        else:                                                      # F works in metres
            self.off.append(1000.0 * np.array([self.f_pred[j][1] for j in range(2)]))
            self.E.append(1000.0 * np.array([self.f_pred[j][0] - self.f_pred[j][2] for j in range(2)]))
        return cmd


def gains(name: str) -> dict:
    if name == M3:
        return dict(horizon_steps=M3_HORIZON, horizon_s=M3_HORIZON * DT, r_u_mm2_per_psi2=M3_RU,
                    dead_time_steps=50, offset_tau_s=0.5, model="M3")
    if name == M2:
        return dict(horizon_steps=100, horizon_s=1.0, r_u_mm2_per_psi2=r_u_tuned(), dead_time_steps=50, offset_tau_s=0.5)
    return dict(horizon_steps=F_HORIZON, horizon_s=F_HORIZON * DT, rho_m2_per_psi2=F_RHO, dead_time_steps=50)


def run(job):
    test, name = job
    fn, seed, settle_s, kind = TESTS[test]
    d = ref_array(fn)
    rec = Recorder(name, d)
    res = track_ref_mm(rec, d, int(round(TRACK_S * HZ)), seed=seed)
    met = score(1000.0 * (res["tips"] - res["home"]), 1000.0 * (res["refs"] - res["home"]), settle_s, res["cmds"], kind)
    m = M2Model.load()
    a = np.array([ax.params(0.0)[0] for ax in m.axes_measured])
    b = np.array([ax.params(0.0)[1] for ax in m.axes_measured])
    p = np.array(rec.p)
    x = 1000.0 * (np.array(rec.x) - res["home"])
    dist = 1000.0 * np.array(rec.v) - (-a * x + b * ((p - p[0]) @ m.B_measured.T))
    E = np.array(rec.E)
    Ed = np.gradient(E, DT, axis=0)
    s = slice(None, None, EVERY)
    return dict(test=test, controller=name, gains=gains(name), metrics=met, settle_s=settle_s,
                t_s=(np.arange(len(p)) * DT)[s].round(3).tolist(), u_psi=res["cmds"][s].round(4).tolist(),
                p_psi=p[s].round(4).tolist(), d_mm_s=dist[s, :2].round(4).tolist(),
                offset_mm=np.array(rec.off)[s].round(5).tolist(), E_mm=E[s].round(5).tolist(),
                E_dot_mm_s=Ed[s].round(5).tolist(),
                tip_mm=(1000.0 * (res["tips"] - res["home"]))[s, :2].round(4).tolist(),
                ref_mm=(1000.0 * (res["refs"] - res["home"]))[s, :2].round(4).tolist())


def figure(rows: list, test: str, path: Path | None = None, names: tuple = (M2, F)):
    R = {r["controller"]: r for r in rows if r["test"] == test}
    fig, axs = P.figure(4, len(names), sharex=True, layout="constrained")
    fig.set_size_inches(P.WIDTH * 0.6 * len(names), P.PANEL_H * 4 * 1.05)
    for j, name in enumerate(names):
        r = R[name]
        t = np.array(r["t_s"])
        for k, seg in enumerate(("S2", "S3", "S4")):
            axs[0, j].plot(t, np.array(r["u_psi"])[:, k], **P.line(seg, len(t)))
            axs[1, j].plot(t, np.array(r["p_psi"])[:, k], **P.line(seg, len(t)))
        for k, axn in enumerate("xy"):
            axs[2, j].plot(t, np.array(r["d_mm_s"])[:, k], **P.line(f"d {axn}", len(t)))
            axs[3, j].plot(t, np.array(r["offset_mm"])[:, k], **P.line(f"E {axn}", len(t)))
        off = "o (Kalman tip offset)" if name == F else "w (low-passed prediction error)"
        axs[0, j].set(title=f"{P.label(name)}\ncommanded pressure u", ylabel="psi", ylim=(-0.3, 9.3))
        axs[1, j].set(title="measured segment pressure p", ylabel="psi")
        axs[2, j].set(title="actual disturbance d (M2 model residual)", ylabel="mm/s")
        axs[3, j].set(title=f"MPC's offset estimate {off}", ylabel="mm", xlabel="tracking time (s)")
        for a in axs[:, j]:
            a.grid(alpha=0.3)
    h0, l0 = axs[0, 0].get_legend_handles_labels()
    h2, l2 = axs[2, 0].get_legend_handles_labels()
    h3, l3 = axs[3, 0].get_legend_handles_labels()
    fig.legend(h0 + h2 + h3, l0 + l2 + [f"offset, {s}" for s in l3], loc="outside lower center", ncol=7)
    fig.suptitle(f"{test}: MPC pressures and disturbance (make_calibrated_sim, seed {TESTS[test][1]})", fontsize=16)
    if path is None:
        return fig
    P.save(fig, path)


def figure_one(rows: list, test: str, name: str):
    """One MPC on one test, 2 x 2: commanded and measured pressure, actual disturbance, offset estimate."""
    r = next(r for r in rows if r["test"] == test and r["controller"] == name)
    fig, axs = P.figure(2, 2, sharex=True, layout="constrained")
    fig.set_size_inches(P.WIDTH * 1.2, P.PANEL_H * 2 * 1.2)
    t = np.array(r["t_s"])
    for k, seg in enumerate(("S2", "S3", "S4")):
        axs[0, 0].plot(t, np.array(r["u_psi"])[:, k], **P.line(seg, len(t)))
        axs[0, 1].plot(t, np.array(r["p_psi"])[:, k], **P.line(seg, len(t)))
    for k, axn in enumerate("xy"):
        axs[1, 0].plot(t, np.array(r["d_mm_s"])[:, k], **P.line(f"d {axn}", len(t)))
        axs[1, 1].plot(t, np.array(r["offset_mm"])[:, k], **P.line(f"E {axn}", len(t), label=f"offset, {axn}"))
    off = "o (Kalman tip offset)" if name == F else "w (low-passed prediction error)"
    axs[0, 0].set(title="commanded pressure u", ylabel="psi", ylim=(-0.3, 9.3))
    axs[0, 1].set(title="measured segment pressure p", ylabel="psi")
    axs[1, 0].set(title="actual disturbance d (M2 model residual)", ylabel="mm/s", xlabel="tracking time (s)")
    axs[1, 1].set(title=f"offset estimate {off}", ylabel="mm", xlabel="tracking time (s)")
    for a in axs.ravel():
        a.grid(alpha=0.3)
    h0, l0 = axs[0, 0].get_legend_handles_labels()
    h2, l2 = axs[1, 0].get_legend_handles_labels()
    h3, l3 = axs[1, 1].get_legend_handles_labels()
    fig.legend(h0 + h2 + h3, l0 + l2 + l3, loc="outside lower center", ncol=7)
    fig.suptitle(f"{test}: {P.label(name)}", fontsize=16)
    return fig


def phase(rows: list, test: str, path: Path | None = None, names: tuple = (M2, F)):
    R = {r["controller"]: r for r in rows if r["test"] == test}
    fig, axs = P.figure(len(names), 2, layout="constrained")
    fig.set_size_inches(P.WIDTH * 1.2, P.PANEL_H * len(names) * 1.5)
    for i, name in enumerate(names):
        r = R[name]
        E, Ed = np.array(r["E_mm"]), np.array(r["E_dot_mm_s"])
        k0 = int(round(r["settle_s"] / (EVERY * DT)))
        for j, sl in enumerate((slice(None), slice(k0, None))):
            ax = axs[i, j]
            for k, axn in enumerate("xy"):
                ax.plot(E[sl, k], Ed[sl, k], **P.line(f"E {axn}", len(E[sl])))
            ax.axhline(0, color="0.6", lw=0.8)
            ax.axvline(0, color="0.6", lw=0.8)
            lim, ylim = 1.15 * np.abs(E[sl]).max(), 1.15 * np.abs(Ed[sl]).max()
            ax.set(xlim=(-lim, lim), ylim=(-ylim, ylim), xlabel="E = predicted tip at t+T + offset − x_d(t+T) (mm)",
                   ylabel="Ė (mm/s)", title=f"{P.label(name)}\n{'whole run' if j == 0 else 'after settling'}")
            ax.grid(alpha=0.3)
    h, lb = axs[-1, 0].get_legend_handles_labels()
    fig.legend(h, lb, loc="outside lower center", ncol=2)
    fig.suptitle(f"{test}: MPC error-space phase portrait (make_calibrated_sim, seed {TESTS[test][1]})", fontsize=16)
    if path is None:
        return fig
    P.save(fig, path)


def run_all(names: tuple = (M2, F)) -> list:
    with mp.get_context("fork").Pool(12) as pool:
        return pool.map(run, [(t, n) for t in TESTS for n in names])


def main() -> None:
    P.apply()
    rows = run_all()
    rms = lambda a: np.sqrt(np.mean(np.asarray(a) ** 2, axis=0))  # noqa: E731
    for t in TESTS:
        figure(rows, t, FIG / f"mpc_signals_{t.replace(' ', '_')}.png")
        phase(rows, t, FIG / f"mpc_phase_{t.replace(' ', '_')}.png")
        for r in (r for r in rows if r["test"] == t):
            d, o = rms(r["d_mm_s"]), rms(r["offset_mm"])
            print(f"{t:<17}{r['controller']:<40} RMSE settled {r['metrics']['rmse_settled_mm']:.3f} mm  "
                  f"|d| rms {d[0]:.2f}/{d[1]:.2f} mm/s  |offset| rms {o[0]:.3f}/{o[1]:.3f} mm")
    (FIG / "mpc_signals.json").write_text(json.dumps(dict(description=__doc__.split("\n\n")[0],
                                                             trace_step_s=EVERY * DT, runs=rows)))
    print("wrote docs/figures/mpc_signals.json, mpc_signals_<test>.png and mpc_phase_<test>.png")


if __name__ == "__main__":
    main()
