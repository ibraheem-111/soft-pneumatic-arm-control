"""Pressures and disturbance inside the delay-aware SMC and NDOSMC on the M2 comparison tests.

Runs the two sliding-mode controllers of compare_m2_controllers.py (reference known in advance)
on its six tests and records, every 10 ms:
  u        commanded segment pressure S2-S4 (psi), what the controller sends
  p        measured segment pressure S2-S4 (psi), what the pouch sensors read
  d_hat    the observer's disturbance estimate (mm/s), NDOSMC only, as used in the law
  d        the disturbance M2 actually sees: d = x_dot - (-a x + b H (p - p0)), from the measured
           tip velocity, tip and pressure (mm/s); x and y axes
Also runs the SMC at lambda = 4 with eta = 1 and eta = 10 and no observer (metrics only), to
separate the NDOSMC's larger switching gain (eta -> eps/l1 = 10) from its disturbance estimate.
d is what the observer tries to estimate: everything M2 does not model (lag/delay mismatch,
coupling, nonlinearity). d_hat acts on the predicted tip x_hat, d on the measured one, so d_hat
leads d by about the 0.5 s dead time.
  E        the error the sliding law acts on: predicted tip x_hat minus r(t+T) (mm); E_dot by
           central differences (mm/s). Phase portrait E vs E_dot with the sliding line
           s = lambda E + E_dot = 0.

Writes docs/figures/smc_signals.json (traces every 50 ms), docs/figures/smc_signals_<test>.png and
docs/figures/smc_phase_<test>.png.

Usage:
    uv run python scripts/smc_signals.py
"""

import json
import multiprocessing as mp
import os
import sys
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import benchmark_all_tasks as B  # noqa: E402
import benchmark_slide_tasks as BS  # noqa: E402
import compare_m2_controllers as C  # noqa: E402
import ndosmc as N  # noqa: E402
import plot_style as P  # noqa: E402

SMC, NDO = "Delay-aware SMC (M2, known ref)", "Delay-aware NDOSMC (M2, known ref)"
L4E1, L4E10 = "SMC λ = 4, η = 1, no observer", "SMC λ = 4, η = 10, no observer"
GAINS = {SMC: dict(lam=8.0, mu=1.0, eps=1.0, observer=False, exact_preview=True),
         NDO: dict(lam=4.0, l1=0.1, l2=0.0, mu=1.0, Phi=1.0, eps=1.0, exact_preview=True),
         # switching-gain check: the NDOSMC's eta = Phi e^(-l1 t) + eps/l1 -> 10, against eta = eps = 1
         L4E1: dict(lam=4.0, mu=1.0, eps=1.0, observer=False, exact_preview=True),
         L4E10: dict(lam=4.0, mu=1.0, eps=10.0, observer=False, exact_preview=True)}
EVERY = 5


class Recorder:
    """Wraps Ahead(DelayAwareNDOSMC) and logs pressures and the observer's estimate each step."""

    def __init__(self, name: str, ref_mm: np.ndarray):
        self.m = N.TaskSpaceModel.load()
        self.c = N.DelayAwareNDOSMC(self.m, **GAINS[name])
        self.ctl = BS.Ahead(self.c, ref_mm)
        self.est: list = []
        est = self.c.obs.estimate
        self.c.obs.estimate = lambda q: self.est.append(est(q)) or self.est[-1]
        self.p, self.v, self.x, self.dh, self.E = [], [], [], [], []
        self.d = ref_mm

    def compute(self, t, obs, ref):
        n, k, c = len(self.est), self.ctl.k, self.c
        tip = np.asarray(obs["tip_pos"], float)
        q = np.zeros(3) if c.home is None else 1000.0 * (tip - c.home)
        x_hat = q + c.x_model - c.x_hist[0]                   # the predicted tip the law uses this step
        self.E.append(x_hat - self.d[min(k + c.n_delay, len(self.d) - 1)])
        cmd = self.ctl.compute(t, obs, ref)
        self.p.append(np.asarray(obs["segment_pressures"], float)[1:4])
        self.x.append(np.asarray(obs["tip_pos"], float))
        self.v.append(np.asarray(obs["tip_vel"], float))
        self.dh.append(self.est[n] if self.c.use_observer else np.zeros(3))
        return cmd


def run(job):
    test, name = job
    fn, seed, settle_s, kind = C.TESTS[test]
    d = C.ref_array(fn)
    rec = Recorder(name, d)
    res = B.track(rec, lambda home, k: home + d[k] / 1000.0, int(round(C.TRACK_S * C.HZ)), seed=seed)
    m = rec.m
    a = np.array([ax.params(0.0)[0] for ax in m.axes_measured])
    b = np.array([ax.params(0.0)[1] for ax in m.axes_measured])
    p = np.array(rec.p)
    x = 1000.0 * (np.array(rec.x) - res["home"])
    xd = 1000.0 * np.array(rec.v)
    dist = xd - (-a * x + b * ((p - p[0]) @ m.B_measured.T))
    met = C.score(1000.0 * (res["tips"] - res["home"]), 1000.0 * (res["refs"] - res["home"]), settle_s, res["cmds"], kind)
    E = np.array(rec.E)
    Ed = np.gradient(E, C.DT, axis=0)
    s = slice(None, None, EVERY)
    return dict(metrics=met, E_mm=E[s, :2].round(5).tolist(), E_dot_mm_s=Ed[s, :2].round(5).tolist(), lam=GAINS[name]["lam"],
                settle_s=C.TESTS[test][2], test=test, controller=name, gains=GAINS[name], t_s=(np.arange(len(p)) * C.DT)[s].round(3).tolist(),
                u_psi=res["cmds"][s].round(4).tolist(), p_psi=p[s].round(4).tolist(),
                d_hat_mm_s=np.array(rec.dh)[s, :2].round(4).tolist(), d_mm_s=dist[s, :2].round(4).tolist())


def figure(rows: list, test: str, path: Path) -> None:
    R = {r["controller"]: r for r in rows if r["test"] == test}
    fig, axs = P.figure(3, 2, sharex=True, layout="constrained")
    fig.set_size_inches(P.WIDTH * 1.2, P.PANEL_H * 3 * 1.15)
    for j, name in enumerate((SMC, NDO)):
        r = R[name]
        t = np.array(r["t_s"])
        for k, seg in enumerate(("S2", "S3", "S4")):
            axs[0, j].plot(t, np.array(r["u_psi"])[:, k], **P.line(seg, len(t)))
            axs[1, j].plot(t, np.array(r["p_psi"])[:, k], **P.line(seg, len(t)))
        for k, axn in enumerate("xy"):
            axs[2, j].plot(t, np.array(r["d_mm_s"])[:, k], **P.line(f"d {axn}", len(t)))
            if name == NDO:
                axs[2, j].plot(t, np.array(r["d_hat_mm_s"])[:, k], **P.line(f"d_hat {axn}", len(t)))
        axs[0, j].set(title=f"{P.label(name)}\ncommanded pressure u", ylabel="psi", ylim=(-0.3, 9.3))
        axs[1, j].set(title="measured segment pressure p", ylabel="psi")
        axs[2, j].set(title="disturbance: actual d (model residual)" + (" and observer d̂" if name == NDO else ""),
                      ylabel="mm/s", xlabel="tracking time (s)")
        for a in axs[:, j]:
            a.grid(alpha=0.3)
    h0, l0 = axs[0, 0].get_legend_handles_labels()
    h2, l2 = axs[2, 1].get_legend_handles_labels()
    fig.legend(h0 + h2, l0 + l2, loc="outside lower center", ncol=7)
    fig.suptitle(f"{test}: pressures and disturbance (make_calibrated_sim, seed {C.TESTS[test][1]})", fontsize=16)
    P.save(fig, path)


def phase(rows: list, test: str, path: Path) -> None:
    """E vs E_dot per controller: whole run (reaching the line) and after settling (sliding)."""
    R = {r["controller"]: r for r in rows if r["test"] == test}
    fig, axs = P.figure(2, 2, layout="constrained")
    fig.set_size_inches(P.WIDTH * 1.2, P.PANEL_H * 2 * 1.5)
    for i, name in enumerate((SMC, NDO)):
        r = R[name]
        E, Ed = np.array(r["E_mm"]), np.array(r["E_dot_mm_s"])
        k0 = int(round(r["settle_s"] / (EVERY * C.DT)))
        g = r["gains"]
        mu = g["mu"]
        eta = g["eps"] / g["l1"] if "l1" in g else g["eps"]          # NDOSMC: Phi e^(-l1 t) + eps/l1 -> eps/l1
        for j, sl in enumerate((slice(None), slice(k0, None))):
            ax = axs[i, j]
            for k, axn in enumerate("xy"):
                ax.plot(E[sl, k], Ed[sl, k], **P.line(f"E {axn}", len(E[sl])))
            lim = 1.15 * np.abs(E[sl]).max()
            e = np.linspace(-lim, lim, 2)
            ax.plot(e, -r["lam"] * e, **{**P.REFERENCE, "label": "sliding line s = λE + Ė = 0"})
            el = np.linspace(-min(lim, mu), min(lim, mu), 2)
            ax.plot(el, -(r["lam"] + eta / mu) * el, color="0.45", ls=":", lw=3,
                    label="closed loop for |E| < μ: Ė = −(λ + η/μ) E")
            ax.axhline(0, color="0.6", lw=0.8)
            ax.axvline(0, color="0.6", lw=0.8)
            ylim = 1.15 * np.abs(Ed[sl]).max()
            ax.set(xlim=(-lim, lim), ylim=(-ylim, ylim), xlabel="E = x̂ − x_d(t+T) (mm)", ylabel="Ė (mm/s)",
                   title=f"{P.label(name)}, λ = {r['lam']:g}, η ≈ {eta:g}\n{'whole run' if j == 0 else 'after settling'}")
            ax.grid(alpha=0.3)
    h, lb = axs[1, 0].get_legend_handles_labels()
    fig.legend(h, lb, loc="outside lower center", ncol=3)
    fig.suptitle(f"{test}: error-space phase portrait (make_calibrated_sim, seed {C.TESTS[test][1]})", fontsize=16)
    P.save(fig, path)


def main() -> None:
    P.apply()
    with mp.get_context("fork").Pool(min(12, os.cpu_count() or 4)) as pool:
        rows = pool.map(run, [(t, n) for t in C.TESTS for n in GAINS])
    for t in C.TESTS:
        figure(rows, t, C.FIG / f"smc_signals_{t.replace(' ', '_')}.png")
        phase(rows, t, C.FIG / f"smc_phase_{t.replace(' ', '_')}.png")
        for r in (r for r in rows if r["test"] == t and r["controller"] in (SMC, NDO)):
            d, dh = np.array(r["d_mm_s"]), np.array(r["d_hat_mm_s"])
            print(f"{t:<17}{r['controller']:<38} |d| rms x {np.sqrt(np.mean(d[:, 0]**2)):6.2f} y "
                  f"{np.sqrt(np.mean(d[:, 1]**2)):6.2f} mm/s   |d_hat| rms x {np.sqrt(np.mean(dh[:, 0]**2)):6.2f} "
                  f"y {np.sqrt(np.mean(dh[:, 1]**2)):6.2f}")
    print("\nswitching-gain check, RMSE whole / after settling (mm):")
    for t in C.TESTS:
        print(f"  {t:<17}" + "  ".join(f"{n}: {r['metrics']['rmse_whole_mm']:.2f}/{r['metrics']['rmse_settled_mm']:.3f}"
                                          for n in (L4E1, L4E10, NDO) for r in rows
                                          if r["test"] == t and r["controller"] == n))
    (C.FIG / "smc_signals.json").write_text(json.dumps(dict(description=__doc__.split("\n\n")[0],
                                                             trace_step_s=EVERY * C.DT, runs=rows)))
    print("wrote docs/figures/smc_signals.json, smc_signals_<test>.png and smc_phase_<test>.png")


if __name__ == "__main__":
    main()
