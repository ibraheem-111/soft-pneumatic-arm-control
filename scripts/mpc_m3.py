"""Delay-compensated MPC on M3 (M2's actuator with second-order mechanics), reference known.

M3 (docs/figures/task_space_model.json M3_mechanics, fitted by scripts/task_space_model.py), per axis:
    tau p' = -p + G u(t - T),    y'' + 2 zeta w y' + w^2 y = w^2 (H p)
State X = (y, y', p) in R^9: tip (mm), tip velocity (mm/s), measured S2-S4 pressure (psi), all
relative to the settled pose. y' is the backward difference of the measured tip (a signal the
hardware has; the simulator's tip_vel is not used). Everything else is compare_m2_controllers.M2MPC:
ZOH at 10 ms, prediction across the 50-step dead time with the commands already sent, offset w
(tip prediction error, low-passed 0.5 s), cost sum |C X(k+N+1+i) + w - r(k+N+i)|^2 + R_u sum |du|^2,
closed-form first move, clipped to 0-9 psi.

Horizon 100 steps = 1 s (as the tuned M2 MPC); R_u swept over compare_m2_controllers.RU_GRID on its
tuning circle (5 mm, 0.07 Hz, seed 5), never on a test.

Usage (module; run by notebooks/mpc_m3.ipynb):
    uv run python scripts/mpc_m3.py        # tune R_u, print it
"""

import json
import multiprocessing as mp
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import expm

sys.path.insert(0, str(Path(__file__).resolve().parent))
import benchmark_all_tasks as B  # noqa: E402
import compare_m2_controllers as C  # noqa: E402
import ndosmc as N  # noqa: E402

HORIZON = 100
M3_LABEL = f"MPC (M3), horizon {HORIZON} steps = {HORIZON * C.DT:.2f} s"


def m3_matrices() -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """Continuous-time M3: (Ac, Bc, C) with state (y, y', p) and the parameters used."""
    act = N.TaskSpaceModel.load().actuator
    mech = json.loads(N.MODEL_JSON.read_text())["M3_mechanics"]
    w = np.array([mech[a]["wn"] for a in "xyz"])
    z = np.array([mech[a]["zeta"] for a in "xyz"])
    H = np.array([mech[a]["H"] for a in "xyz"])
    O, I = np.zeros((3, 3)), np.eye(3)
    Ac = np.block([[O, I, O], [-np.diag(w**2), -np.diag(2 * z * w), np.diag(w**2) @ H], [O, O, -I / act.tau_s]])
    Bc = np.vstack([O, O, np.diag(act.gain) / act.tau_s])
    Cm = np.hstack([I, O, O])
    return Ac, Bc, Cm, dict(wn=w.tolist(), zeta=z.tolist(), H=H.tolist(), tau_s=act.tau_s, gain=list(act.gain),
                            delay_s=act.delay_s)


class DelayMPC:
    """M2MPC's law for any linear model (Ac, Bc, C) whose output is the tip; state built by state_fn."""

    def __init__(self, Ac, Bc, Cm, ref_mm: np.ndarray, horizon: int, r_u: float, delay_steps: int,
                 offset_tau_s: float = 0.5):
        n = Ac.shape[0]
        E = expm(np.block([[Ac, Bc], [np.zeros((3, n + 3))]]) * C.DT)
        self.A, self.B, self.C = E[:n, :n], E[:n, n:], Cm
        self.n, self.nd, self.h, self.r_u, self.d = n, delay_steps, horizon, r_u, ref_mm
        self.alpha = 1.0 - np.exp(-C.DT / offset_tau_s)
        h = horizon
        Apow = [np.eye(n)]
        for _ in range(max(h, self.nd) + 1):
            Apow.append(self.A @ Apow[-1])
        self.Apow = Apow
        Phi = np.vstack([Cm @ Apow[i + 1] for i in range(h)])
        Gam = np.zeros((3 * h, 3 * h))
        for i in range(h):
            for j in range(i + 1):
                Gam[3 * i:3 * i + 3, 3 * j:3 * j + 3] = Cm @ Apow[i - j] @ self.B
        D = np.eye(3 * h) - np.eye(3 * h, k=-3)
        K = np.linalg.inv(Gam.T @ Gam + r_u * D.T @ D)[:3]
        self.g_r = K @ Gam.T
        self.g_x = -self.g_r @ Phi
        self.g_w = -self.g_r @ np.tile(np.eye(3), (h, 1))
        E1 = np.zeros((3 * h, 3))
        E1[:3] = np.eye(3)
        self.g_u = r_u * K @ D.T @ E1
        self.W = [Apow[self.nd - 1 - i] @ self.B for i in range(self.nd)]
        self.k = 0
        self.home = self.p0 = self.y_prev = None
        self.buf = [np.zeros(3)] * self.nd
        self.u_prev = np.zeros(3)
        self.w = np.zeros(3)
        self.pred_hist: list = []

    def state(self, obs: dict) -> np.ndarray:
        """M3 state (y, y', p); y' by backward difference of the measured tip."""
        y = 1000.0 * (np.asarray(obs["tip_pos"], float) - self.home)
        yd = np.zeros(3) if self.y_prev is None else (y - self.y_prev) / C.DT
        self.y_prev = y
        return np.concatenate([y, yd, np.asarray(obs["segment_pressures"], float)[1:4] - self.p0])

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        if self.home is None:
            self.home = np.asarray(obs["tip_pos"], float).copy()
            self.p0 = np.asarray(obs["segment_pressures"], float)[1:4].copy()
        X = self.state(obs)
        y = self.C @ X
        Xf = self.Apow[self.nd] @ X + sum(Wi @ ui for Wi, ui in zip(self.W, self.buf))
        self.pred_hist.append(self.C @ Xf)
        if len(self.pred_hist) > self.nd:
            self.w += self.alpha * ((y - self.pred_hist.pop(0)) - self.w)
        k, h, d = self.k, self.h, self.d
        r = d[min(k + self.nd, len(d) - h):][:h]
        r = np.vstack([r, np.repeat(r[-1:], h - len(r), axis=0)]) if len(r) < h else r
        u = self.g_r @ r.reshape(-1) + self.g_x @ Xf + self.g_w @ self.w + self.g_u @ self.u_prev
        cmd = np.clip(4.5 + u, 0.0, 9.0)
        u_app = cmd - 4.5
        self.buf = self.buf[1:] + [u_app]
        self.u_prev = u_app
        self.k += 1
        return cmd


def make_m3_mpc(ref_mm: np.ndarray, r_u: float, horizon: int = HORIZON) -> DelayMPC:
    Ac, Bc, Cm, prm = m3_matrices()
    ctl = DelayMPC(Ac, Bc, Cm, ref_mm, horizon, r_u, int(round(prm["delay_s"] * C.HZ)))
    ctl.gains = dict(horizon_steps=horizon, horizon_s=horizon * C.DT, r_u=r_u, dead_time_steps=ctl.nd,
                     offset_tau_s=0.5, model="M3 (task_space_model.json M3_mechanics)")
    return ctl


def _tune_job(r_u: float) -> tuple[float, float]:
    d = C.ref_array(C.circle(C.TUNE["radius_m"], C.TUNE["frequency_hz"]))
    res = B.track(make_m3_mpc(d, r_u), lambda home, k: home + d[k] / 1000.0, int(round(C.TRACK_S * C.HZ)),
                  seed=C.TUNE["seed"])
    e = 1000.0 * np.linalg.norm((res["tips"] - res["refs"])[:, :2], axis=1)
    return r_u, float(np.sqrt(np.mean(e[int(round(C.HZ / C.TUNE["frequency_hz"])):] ** 2)))


def tune() -> tuple[float, list]:
    """R_u for the M3 MPC on the tuning circle: (best R_u, [(R_u, RMSE after first lap mm)])."""
    with mp.get_context("fork").Pool(len(C.RU_GRID)) as pool:
        out = pool.map(_tune_job, C.RU_GRID)
    return min(out, key=lambda x: x[1])[0], out


def tracking_figure(rows: list, test: str, names: tuple):
    """Tip path (last lap or after 5 s) and planar error over time for each MPC on one test."""
    import plot_style as P
    R = {r["controller"]: r for r in rows if r["test"] == test}
    _, _, settle_s, kind = C.TESTS[test]
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
    fig.suptitle(f"{test} (make_calibrated_sim, seed {C.TESTS[test][1]})", fontsize=16)
    return fig


if __name__ == "__main__":
    best, grid = tune()
    print("M3 MPC R_u tuning (RMSE after first lap, mm): " + "  ".join(f"{r:g}: {e:.4f}" for r, e in grid))
    print(f"chosen R_u = {best:g}")
