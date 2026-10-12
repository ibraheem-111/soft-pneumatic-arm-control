"""Identify a task-space dynamic model of the calibrated arm from input/output data only.

The plant is `make_calibrated_sim(control_hz=100, seed=s)` treated as the physical
robot: three commanded segment pressures (S2, S3, S4), S1 a sealed reservoir. Only
hardware-available signals are used: `time`, `tip_pos` (motion capture, with added
Gaussian noise, sigma = 0.1 mm per axis, rng seed 1000 + sim seed) and the pressure
sensors (`segment_pressures`, i.e. S1 reservoir + S2..S4). `obs["q"]`,
`obs["p_actual"]`, `tip_vel`, `sim.cfg` and the MuJoCo model are never read.

State X = tip position minus the "home" pose (mean noisy tip over the last 0.5 s of a
5 s settle at 4.5 psi on S2-S4, as in the package's CircularTrackingTask), in mm.
Input u = command - 4.5 psi.

Models (structure and derivation: docs/reports/task-space-model.html):
  M1      Qiao et al. 2024 style first-order LPV, command in, inner loop ideal:
          c_i(rho_i) dX_i/dt + k_i(rho_i) X_i = v_i,  v = G u   (G: 3x3 static gain)
          c_i, k_i identified at 5 set points of the measured scheduling variable
          rho_x = p_S1 - p_S3, rho_y = p_S2 - p_S4, rho_z = p_S1+p_S2+p_S3+p_S4,
          then fitted with quadratics. M1-LTI uses the averaged values (the paper's
          "baseline model").
  M2      actuator (per segment: gain, dead time, first-order lag; command -> measured
          pressure) in series with first-order mechanics per axis (measured pressure
          -> X) plus the geometric z lift kappa (X_x^2 + X_y^2).
  M3      as M2 with second-order (lumped-mass) mechanics per axis.

Experiments (seeds): set points 1, broadband random steps 2, sine check 3, regulator
staircase 4; validation 11 (multisine), 12 (open-loop circle + slow z), 13 (random
steps, other statistics). Scores are free-run simulation RMSE per axis over the whole
validation run, against the noisy measurement.

Writes docs/figures/task_space_*.png, docs/figures/task_space_model.json, and refreshes
the embedded figures (<img data-fig="...">) in docs/reports/task-space-model.html if
that file exists.

Usage:
    uv run python scripts/task_space_model.py            # identify, validate, write all outputs
    uv run python scripts/task_space_model.py --embed-only   # only re-embed figures in the report
"""

from __future__ import annotations

import argparse
import base64
import json
import re
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import cont2discrete, lfilter, savgol_filter

from soft_robotic_arm import make_calibrated_sim

ROOT = Path(__file__).resolve().parents[1]
FIG_DIR = ROOT / "docs" / "figures"
REPORT = ROOT / "docs" / "reports" / "task-space-model.html"

HZ = 100
DT = 1.0 / HZ
U0 = 4.5  # psi, baseline on S2..S4
SETTLE_S = 5.0
TIP_NOISE_M = 1e-4  # 0.1 mm per axis
SEG = ["S2", "S3", "S4"]
AX = ["x", "y", "z"]

SEED_SETPOINT, SEED_BROADBAND, SEED_SINE, SEED_REGULATOR = 1, 2, 3, 4
SEED_VAL = {"V1 multisine": 11, "V2 open-loop circle": 12, "V3 random steps": 13}

# Set-point design for M1: direction in (S2, S3, S4) command space, five levels, doublet size.
SP_DESIGN = {
    "x": (np.array([0.0, 1.0, 0.0]), [-2.5, -1.25, 0.0, 1.25, 2.5], 1.0),
    "y": (np.array([1.0, 0.0, -1.0]), [-2.0, -1.0, 0.0, 1.0, 2.0], 0.75),
    "z": (np.array([1.0, 1.0, 1.0]), [-2.5, -1.25, 0.0, 1.25, 2.5], 1.0),
}
SP_HOLD, SP_STEP, SP_BACK = 8.0, 8.0, 6.0  # s


# ----------------------------------------------------------------------------- data


@dataclass
class Record:
    t: np.ndarray  # (N,) s since end of settle
    du: np.ndarray  # (N, 3) command deviation (psi) on S2..S4
    p: np.ndarray  # (N, 4) measured segment pressures S1..S4 (psi)
    dp: np.ndarray  # (N, 4) measured pressure minus home pressure
    X: np.ndarray  # (N, 3) noisy tip deviation from home (mm)
    X_clean: np.ndarray  # (N, 3) same without the added noise (scoring only, never fitted)
    p_home: np.ndarray  # (4,)
    home_mm: np.ndarray  # (3,) noisy home tip (mm, mocap frame)


def run_experiment(du: np.ndarray, seed: int) -> Record:
    """Settle 5 s at 4.5 psi, then apply command deviations du (N, 3); record sensors."""
    sim = make_calibrated_sim(control_hz=HZ, seed=seed)
    rng = np.random.default_rng(1000 + seed)
    base = np.full(3, U0)
    n_settle = int(SETTLE_S * HZ)
    tips, clean, pres = [], [], []
    for k in range(n_settle + len(du)):
        cmd = base if k < n_settle else np.clip(base + du[k - n_settle], 0.0, 9.0)
        obs = sim.step(cmd)
        clean.append(np.array(obs["tip_pos"]))
        tips.append(clean[-1] + rng.normal(0.0, TIP_NOISE_M, 3))
        pres.append(np.asarray(obs["segment_pressures"], float))
    sim.close()
    tips = np.array(tips) * 1e3
    pres = np.array(pres)
    home = tips[n_settle - 50:n_settle].mean(0)
    p_home = pres[n_settle - 50:n_settle].mean(0)
    p = pres[n_settle:]
    return Record(
        t=np.arange(len(du)) * DT,
        du=np.clip(base + du, 0.0, 9.0) - base,
        p=p,
        dp=p - p_home,
        X=tips[n_settle:] - home,
        X_clean=np.array(clean)[n_settle:] * 1e3 - home,
        p_home=p_home,
        home_mm=home,
    )


def setpoint_sequence(axis: str, level: float) -> np.ndarray:
    direction, _, delta = SP_DESIGN[axis]
    seq = [level] * int(SP_HOLD * HZ) + [level + delta] * int(SP_STEP * HZ)
    seq += [level - delta] * int(SP_STEP * HZ) + [level] * int(SP_BACK * HZ)
    return np.outer(seq, direction)


def random_steps(duration: float, lo: float, hi: float, hold: tuple[float, float], seed: int) -> np.ndarray:
    """Independent random multilevel steps on each segment (absolute levels lo..hi psi)."""
    rng = np.random.default_rng(seed)
    n = int(duration * HZ)
    out = np.zeros((n, 3))
    for j in range(3):
        k = 0
        while k < n:
            m = int(rng.uniform(*hold) * HZ)
            out[k:k + m, j] = rng.uniform(lo, hi) - U0
            k += m
    return out


def multisine(duration: float, amp: float, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    t = np.arange(int(duration * HZ)) * DT
    freqs = np.geomspace(0.03, 1.5, 36)
    out = np.zeros((len(t), 3))
    for j in range(3):
        fj = freqs[j::3]  # interleaved grids so the segments are uncorrelated
        s = sum(np.sin(2 * np.pi * f * t + rng.uniform(0, 2 * np.pi)) for f in fj)
        out[:, j] = amp * s / np.max(np.abs(s))
    return out


def open_loop_circle(duration: float) -> np.ndarray:
    t = np.arange(int(duration * HZ)) * DT
    a, w = 2.5, 2 * np.pi * 0.1
    zc = 1.0 * np.sin(2 * np.pi * 0.05 * t)
    return np.stack([a * np.sin(w * t) + zc, -a * np.cos(w * t) + zc, -a * np.sin(w * t) + zc], 1)


# ------------------------------------------------------------------- model blocks


def lag1(w: np.ndarray, a: float) -> np.ndarray:
    """Exact discretization of dx/dt = -a x + a w (unit DC gain), zero initial state."""
    phi = np.exp(-a * DT)
    return lfilter([1 - phi], [1, -phi], w)


def lag2(w: np.ndarray, wn: float, zeta: float) -> np.ndarray:
    """ZOH discretization of wn^2 / (s^2 + 2 zeta wn s + wn^2), zero initial state."""
    b, a, _ = cont2discrete(([wn**2], [1, 2 * zeta * wn, wn**2]), DT, method="zoh")
    return lfilter(np.ravel(b), np.ravel(a), w)


def delay(w: np.ndarray, n: int) -> np.ndarray:
    if n == 0:
        return w.copy()
    return np.r_[np.zeros(n), w[:-n]]


def actuator(du: np.ndarray, par: dict) -> np.ndarray:
    """Command deviation (N, 3) -> predicted measured pressure deviation (N, 3)."""
    out = np.zeros_like(du)
    for j in range(3):
        g, tau, n = par["gain"][j], par["tau"][j], par["dead_steps"][j]
        out[:, j] = g * lag1(delay(du[:, j], n), 1.0 / tau)
    return out


# -------------------------------------------------------------------- identification


def fit_actuator(rec: Record) -> dict:
    """Per segment: Delta p = g e^{-Ts} / (tau s + 1) Delta u. Grid over T, LS over (g, tau)."""
    par = {"gain": [], "tau": [], "dead_steps": [], "dead_s": [], "rmse_psi": [], "second_lag_s": [],
           "rmse_psi_two_lags": []}
    for j in range(3):
        u, y = rec.du[:, j], rec.dp[:, j + 1]
        best = None
        for n in range(0, 101):
            ud = delay(u, n)

            def res(th):
                return th[0] * lag1(ud, np.exp(th[1])) - y

            sol = least_squares(res, [1.0, np.log(1 / 0.5)])
            cost = np.mean(sol.fun**2)
            if best is None or cost < best[0]:
                best = (cost, n, sol.x)
        cost, n, th = best
        par["gain"].append(float(th[0]))
        par["tau"].append(float(1.0 / np.exp(th[1])))
        par["dead_steps"].append(int(n))
        par["dead_s"].append(n * DT)
        par["rmse_psi"].append(float(np.sqrt(cost)))

        # Does the data want a second lag? g / ((tau1 s+1)(tau2 s+1)), dead time re-searched nearby.
        best2 = None
        for n2 in range(max(0, n - 15), n + 3):
            ud = delay(u, n2)

            def res2(th):
                return th[0] * lag1(lag1(ud, np.exp(th[1])), np.exp(th[2])) - y

            sol = least_squares(res2, [th[0], th[1], np.log(1 / 0.05)])
            c2 = np.mean(sol.fun**2)
            if best2 is None or c2 < best2[0]:
                best2 = (c2, n2, sol.x)
        taus = sorted(1 / np.exp(best2[2][1:]))
        par["second_lag_s"].append(float(taus[0]))
        par["rmse_psi_two_lags"].append(float(np.sqrt(best2[0])))
    return par


def regulator_static(seed: int) -> dict:
    """Staircase per segment; steady measured pressure vs command."""
    levels = [0.0, 0.25, 0.5, 0.75, 1.0, 2.0, 3.0, 4.5, 6.0, 7.0, 8.0, 9.0]
    hold = int(5 * HZ)
    out = {"levels": levels, "measured": [], "affine_gain": [], "affine_bias": []}
    for j in range(3):
        du = np.zeros((hold * len(levels), 3))
        for i, lv in enumerate(levels):
            du[i * hold:(i + 1) * hold, j] = lv - U0
        rec = run_experiment(du, seed)
        meas = [float(rec.p[(i + 1) * hold - 100:(i + 1) * hold, j + 1].mean()) for i in range(len(levels))]
        lv = np.array(levels)
        m = np.array(meas)
        sel = lv >= 1.0
        g, b = np.polyfit(lv[sel], m[sel], 1)
        out["measured"].append(meas)
        out["affine_gain"].append(float(g))
        out["affine_bias"].append(float(b))
    return out


def steady(rec: Record, k_end: int, span: float = 2.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    sl = slice(k_end - int(span * HZ), k_end)
    return rec.X[sl].mean(0), rec.du[sl].mean(0), rec.p[sl].mean(0)


def rho_of(p: np.ndarray) -> np.ndarray:
    """Scheduling variables from measured pressures (..., 4) = S1..S4."""
    p = np.atleast_2d(p)
    return np.stack([p[:, 0] - p[:, 2], p[:, 1] - p[:, 3], p.sum(1)], 1)


def identify_m1(sp_runs: dict) -> dict:
    """Static gain G, then local first-order (a, K) at five set points per axis."""
    ks = [int(SP_HOLD * HZ), int((SP_HOLD + SP_STEP) * HZ), int((SP_HOLD + 2 * SP_STEP) * HZ)]
    Xs, Us = [], []
    for runs in sp_runs.values():
        for rec in runs:
            for k in ks:
                x, u, _ = steady(rec, k)
                Xs.append(x)
                Us.append(u)
    Xs, Us = np.array(Xs), np.array(Us)
    G = np.linalg.lstsq(Us, Xs, rcond=None)[0].T  # X = G u
    static_rmse = np.sqrt(np.mean((Us @ G.T - Xs) ** 2, 0))

    local = {}
    for i, axis in enumerate(AX):
        rows = []
        for rec in sp_runs[axis]:
            k0 = ks[0]
            x_sp, u_sp, p_sp = steady(rec, k0)
            sl = slice(k0 - HZ, None)
            xl = rec.X[sl, i] - x_sp[i]
            vl = (rec.du[sl] - u_sp) @ G[i]

            def res(th):
                return th[1] * lag1(vl, np.exp(th[0])) - xl

            sol = least_squares(res, [0.0, 1.0])
            a, K = float(np.exp(sol.x[0])), float(sol.x[1])
            rows.append({"rho": float(rho_of(p_sp)[0, i]), "a": a, "K": K, "k": 1 / K, "c": 1 / (a * K),
                         "rmse_mm": float(np.sqrt(np.mean(sol.fun**2)))})
        rho = np.array([r["rho"] for r in rows])
        kk = np.array([r["k"] for r in rows])
        cc = np.array([r["c"] for r in rows])
        local[axis] = {
            "points": rows,
            "k_poly": np.polyfit(rho, kk, 2).tolist(),  # h2, h1, h0
            "c_poly": np.polyfit(rho, cc, 2).tolist(),
            "k_mean": float(kk.mean()),
            "c_mean": float(cc.mean()),
            "rho_range": [float(rho.min()), float(rho.max())],
            "a_spread_pct": float(100 * (np.ptp([r["a"] for r in rows]) / np.mean([r["a"] for r in rows]))),
            "K_spread_pct": float(100 * (np.ptp([r["K"] for r in rows]) / np.mean([r["K"] for r in rows]))),
        }
    c_mean = np.array([local[a]["c_mean"] for a in AX])
    B = np.diag(1 / c_mean) @ G  # paper form: dX/dt = -diag(k/c) X + B u
    return {"G": G.tolist(), "static_rmse_mm": static_rmse.tolist(), "B_lti": B.tolist(), "local": local}


def simulate_m1(m1: dict, rec: Record, lpv: bool) -> np.ndarray:
    G = np.array(m1["G"])
    v = rec.du @ G.T
    out = np.zeros_like(v)
    if not lpv:
        for i, axis in enumerate(AX):
            L = m1["local"][axis]
            out[:, i] = lag1(v[:, i] / L["k_mean"], L["k_mean"] / L["c_mean"])
        return out
    rho = rho_of(rec.p)
    for i, axis in enumerate(AX):
        L = m1["local"][axis]
        r = np.clip(rho[:, i], *L["rho_range"])
        k = np.polyval(L["k_poly"], r)
        c = np.polyval(L["c_poly"], r)
        phi = np.exp(-(k / c) * DT)
        x = 0.0
        for n in range(len(r)):
            x = phi[n] * x + (1 - phi[n]) * v[n, i] / k[n]
            out[n, i] = x
    return out


def fit_m1_broadband(m1: dict, rec: Record) -> list[float]:
    """Reference only: the best first-order, no-delay command model (a_i) on broadband data."""
    G = np.array(m1["G"])
    v = rec.du @ G.T
    out = []
    for i in range(3):
        sol = least_squares(lambda th: lag1(v[:, i], np.exp(th[0])) - rec.X[:, i], [0.0])
        out.append(float(np.exp(sol.x[0])))
    return out


def fit_mechanics(rec: Record, order: int) -> dict:
    """Measured pressure (S2..S4) -> X per axis; first- or second-order, plus z lift."""
    w = rec.dp[:, 1:]
    axes = {}
    pred = np.zeros_like(rec.X)
    for i, axis in enumerate(AX):
        H0 = np.linalg.lstsq(w, rec.X[:, i], rcond=None)[0]
        if order == 1:
            def model(th):
                return lag1(w @ th[1:4], np.exp(th[0]))
            th0 = np.r_[np.log(5.0), H0]
        else:
            def model(th):
                return lag2(w @ th[2:5], np.exp(th[0]), np.exp(th[1]))
            th0 = np.r_[np.log(2 * np.pi * 1.0), np.log(0.6), H0]
        if axis == "z":
            lift = pred[:, 0] ** 2 + pred[:, 1] ** 2

            def res(th):
                return model(th[:-1]) + th[-1] * lift - rec.X[:, i]
            th0 = np.r_[th0, 0.0]
        else:
            def res(th):
                return model(th) - rec.X[:, i]
        sol = least_squares(res, th0)
        th = sol.x
        d = {"rmse_fit_mm": float(np.sqrt(np.mean(sol.fun**2)))}
        if order == 1:
            d.update(a=float(np.exp(th[0])), H=th[1:4].tolist())
            pred[:, i] = model(th[:4]) if axis != "z" else model(th[:4]) + th[-1] * lift
        else:
            d.update(wn=float(np.exp(th[0])), zeta=float(np.exp(th[1])), H=th[2:5].tolist())
            pred[:, i] = model(th[:5]) if axis != "z" else model(th[:5]) + th[-1] * lift
        if axis == "z":
            d["kappa_per_mm"] = float(th[-1])
        axes[axis] = d
    return axes


def fit_x_with_reservoir(rec: Record) -> dict:
    """Second-order x mechanics with the measured S1 reservoir pressure as a fourth input."""
    def model(th, w):
        return lag2(w @ th[2:6], np.exp(th[0]), np.exp(th[1]))
    sol = least_squares(lambda th: model(th, rec.dp) - rec.X[:, 0], np.r_[np.log(7.6), np.log(0.7), 0, -1.8, 0, 0])
    return {"wn": float(np.exp(sol.x[0])), "zeta": float(np.exp(sol.x[1])), "H_S1_S4": sol.x[2:].tolist(),
            "rmse_fit_mm": float(np.sqrt(np.mean(sol.fun**2)))}


def simulate_x_with_reservoir(m: dict, dp: np.ndarray) -> np.ndarray:
    return lag2(dp @ np.array(m["H_S1_S4"]), m["wn"], m["zeta"])


def simulate_mech(mech: dict, w: np.ndarray, order: int, lift: bool = True) -> np.ndarray:
    out = np.zeros((len(w), 3))
    for i, axis in enumerate(AX):
        m = mech[axis]
        u = w @ np.array(m["H"])
        out[:, i] = lag1(u, m["a"]) if order == 1 else lag2(u, m["wn"], m["zeta"])
    if lift:
        out[:, 2] += mech["z"]["kappa_per_mm"] * (out[:, 0] ** 2 + out[:, 1] ** 2)
    return out


def sine_check(seed: int, freqs: list[float], amp: float = 3.0) -> list[dict]:
    """S3 sine tests: complex ratio X_x / measured Delta p_S3 at each frequency."""
    out = []
    for f in freqs:
        n_tr = int(max(10.0, 3 / f) * HZ)
        n_an = int(max(20.0, 4 / f) * HZ)
        n_an = int(round(n_an * f * DT) / (f * DT))  # whole periods
        t = np.arange(n_tr + n_an) * DT
        du = np.zeros((len(t), 3))
        du[:, 1] = amp * np.sin(2 * np.pi * f * t)
        rec = run_experiment(du, seed)
        sl = slice(n_tr, None)
        e = np.exp(-2j * np.pi * f * rec.t[sl])
        px = np.mean((rec.dp[sl, 2] - rec.dp[sl, 2].mean()) * e)
        xx = np.mean((rec.X[sl, 0] - rec.X[sl, 0].mean()) * e)
        out.append({"f_hz": f, "ratio_re": float((xx / px).real), "ratio_im": float((xx / px).imag)})
    return out


def disturbance_stats(X: np.ndarray, a: np.ndarray, v: np.ndarray) -> dict:
    """d_i = dX_i/dt + a_i X_i - a_i v_i, with dX/dt from Savitzky-Golay on noisy X."""
    xd = savgol_filter(X, 31, 3, deriv=1, delta=DT, axis=0)
    d = xd + a * X - a * v
    dd = savgol_filter(d, 51, 2, deriv=1, delta=DT, axis=0)
    return {"rms": np.sqrt(np.mean(d**2, 0)).tolist(), "p99_abs": np.percentile(np.abs(d), 99, 0).tolist(),
            "p99_abs_rate": np.percentile(np.abs(dd), 99, 0).tolist()}


def rmse(a: np.ndarray, b: np.ndarray) -> list[float]:
    return np.sqrt(np.mean((a - b) ** 2, 0)).tolist()


# --------------------------------------------------------------------------- figures

INK, MUTED, GRID = "#1d232b", "#5c6773", "#d9dee4"
C_M1, C_M1L, C_M2, C_M3 = "#eb6834", "#eda100", "#2a78d6", "#1baf7a"


def style():
    plt.rcParams.update({
        "figure.dpi": 110, "savefig.dpi": 150, "font.size": 9, "axes.edgecolor": MUTED,
        "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED, "axes.grid": True,
        "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
        "legend.frameon": False, "lines.linewidth": 1.4, "figure.facecolor": "white",
    })


def fig_actuator(rec: Record, act: dict, reg: dict):
    pred = actuator(rec.du, act)
    fig, axs = plt.subplots(1, 3, figsize=(10, 3.0), gridspec_kw={"width_ratios": [2.2, 1.2, 1.3]})
    sl = slice(int(20 * HZ), int(45 * HZ))
    for j, c in zip(range(3), [C_M2, C_M1, C_M3]):
        axs[0].plot(rec.t[sl], rec.dp[sl, j + 1] + rec.p_home[j + 1], color=c, lw=1.6, label=f"{SEG[j]} measured")
        axs[0].plot(rec.t[sl], pred[sl, j] + rec.p_home[j + 1], color=INK, lw=0.9, ls="--")
    axs[0].plot([], [], color=INK, lw=0.9, ls="--", label="actuator model")
    axs[0].set(xlabel="time (s)", ylabel="pressure (psi)", title="Broadband data (seed 2), excerpt")
    axs[0].legend(ncol=2, fontsize=8, loc="upper left")
    # zoom on the first S3 step after 20 s
    du3 = rec.du[:, 1]
    k = np.flatnonzero(np.abs(np.diff(du3[int(20 * HZ):])) > 0.5)[0] + int(20 * HZ) + 1
    z = slice(k - 50, k + 300)
    tt = (np.arange(z.stop - z.start) - 50) * DT
    axs[1].plot(tt, du3[z] + U0, color=MUTED, lw=1.0, drawstyle="steps-post", label="command")
    axs[1].plot(tt, rec.p[z, 2], color=C_M1, lw=1.6, label="measured")
    axs[1].plot(tt, pred[z, 1] + rec.p_home[2], color=INK, lw=0.9, ls="--", label="model")
    axs[1].axvline(act["dead_s"][1], color=MUTED, lw=0.8, ls=":")
    axs[1].set(xlabel="time from S3 step (s)", ylabel="psi", title=f"S3 step: dead time {act['dead_s'][1]:.2f} s")
    axs[1].legend(fontsize=8)
    lv = np.array(reg["levels"])
    for j, c in zip(range(3), [C_M2, C_M1, C_M3]):
        axs[2].plot(lv, reg["measured"][j], "o", ms=4, color=c, label=SEG[j])
        axs[2].plot(lv[lv >= 1], reg["affine_gain"][j] * lv[lv >= 1] + reg["affine_bias"][j], color=c, lw=0.8)
    axs[2].set(xlabel="command (psi)", ylabel="steady measured (psi)", title="Regulator staircase (seed 4)")
    axs[2].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "task_space_actuator.png")
    plt.close(fig)


def fig_scheduling(m1: dict):
    fig, axs = plt.subplots(2, 3, figsize=(10, 4.6), sharex="col")
    names = {"x": r"$\rho_x = p_{S1}-p_{S3}$", "y": r"$\rho_y = p_{S2}-p_{S4}$", "z": r"$\rho_z = \Sigma p_s$"}
    for i, axis in enumerate(AX):
        L = m1["local"][axis]
        rho = np.array([p["rho"] for p in L["points"]])
        rr = np.linspace(rho.min(), rho.max(), 50)
        for row, key, poly, lab in [(0, "k", "k_poly", "$k_i$ (-)"), (1, "c", "c_poly", "$c_i$ (s)")]:
            ax = axs[row, i]
            vals = [p[key] for p in L["points"]]
            ax.plot(rho, vals, "o", ms=5, color=C_M1, label="local fit at set point")
            ax.plot(rr, np.polyval(L[poly], rr), color=INK, lw=1.0, label="quadratic in ρ")
            ax.axhline(L[f"{key}_mean"], color=MUTED, lw=0.8, ls="--", label="mean (M1-LTI)")
            m = L[f"{key}_mean"]
            ax.set_ylim(m * 0.8, m * 1.2)
            ax.set_ylabel(lab)
            if row == 0:
                ax.set_title(f"axis {axis}")
            else:
                ax.set_xlabel(names[axis] + " (psi)")
    axs[0, 0].legend(fontsize=7, loc="lower left")
    fig.suptitle("M1 local parameters at five set points (y-limits ±20 % of the mean)", fontsize=9, color=INK)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "task_space_scheduling.png")
    plt.close(fig)


def fig_freq(sines: list[dict], m2m: dict, m3m: dict):
    f = np.array([s["f_hz"] for s in sines])
    r = np.array([s["ratio_re"] + 1j * s["ratio_im"] for s in sines])
    ff = np.geomspace(0.05, 3, 200)
    s = 2j * np.pi * ff
    h1 = m2m["x"]["H"][1] * m2m["x"]["a"] / (s + m2m["x"]["a"])
    wn, ze = m3m["x"]["wn"], m3m["x"]["zeta"]
    h2 = m3m["x"]["H"][1] * wn**2 / (s**2 + 2 * ze * wn * s + wn**2)
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.0))
    axs[0].loglog(f, np.abs(r), "o", color=INK, ms=5, label="sine tests (seed 3)")
    axs[0].loglog(ff, np.abs(h1), color=C_M2, label="M2 first order")
    axs[0].loglog(ff, np.abs(h2), color=C_M3, label="M3 second order")
    axs[0].set(xlabel="frequency (Hz)", ylabel="|X_x / Δp_S3| (mm/psi)")
    axs[0].legend(fontsize=8)
    ph = lambda h: np.degrees(np.unwrap(np.angle(-h)))  # noqa: E731  (S3 pushes -x; drop the sign)
    axs[1].semilogx(f, ph(r), "o", color=INK, ms=5)
    axs[1].semilogx(ff, ph(h1), color=C_M2)
    axs[1].semilogx(ff, ph(h2), color=C_M3)
    axs[1].set(xlabel="frequency (Hz)", ylabel="phase of −X_x/Δp_S3 (deg)")
    fig.suptitle("Mechanics only: measured S3 pressure → tip x", fontsize=9, color=INK)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "task_space_freqresp.png")
    plt.close(fig)


def fig_validation(name: str, rec: Record, preds: dict, fname: str):
    fig, axs = plt.subplots(3, 1, figsize=(10, 6.4), sharex=True)
    cols = {"M1-LPV": C_M1, "M2": C_M2, "M3": C_M3}
    for i, axis in enumerate(AX):
        ax = axs[i]
        ax.plot(rec.t, rec.X[:, i], color="#9aa4ae", lw=1.0, label="measured (noisy)")
        for k, c in cols.items():
            ax.plot(rec.t, preds[k][:, i], color=c, lw=1.3, label=k)
        ax.set_ylabel(f"{axis} (mm)")
    axs[0].legend(ncol=4, fontsize=8, loc="upper right")
    axs[-1].set_xlabel("time (s)")
    fig.suptitle(f"Free-run validation, {name} (seed {SEED_VAL[name]})", fontsize=9, color=INK)
    fig.tight_layout()
    fig.savefig(FIG_DIR / fname)
    plt.close(fig)


def fig_lift(sp_runs: dict, kappa: float):
    xs, zs = [], []
    ks = [int(SP_HOLD * HZ), int((SP_HOLD + SP_STEP) * HZ), int((SP_HOLD + 2 * SP_STEP) * HZ)]
    for rec in sp_runs["x"] + sp_runs["y"]:
        for k in ks:
            x, _, _ = steady(rec, k)
            xs.append(x)
    xs = np.array(xs)
    # remove the linear (pressure-sum) part: regress z on (x, y, r^2, 1)
    r2 = xs[:, 0] ** 2 + xs[:, 1] ** 2
    A = np.c_[xs[:, 0], xs[:, 1], r2, np.ones(len(xs))]
    coef = np.linalg.lstsq(A, xs[:, 2], rcond=None)[0]
    zres = xs[:, 2] - A[:, [0, 1, 3]] @ coef[[0, 1, 3]]
    fig, ax = plt.subplots(figsize=(4.4, 3.0))
    ax.plot(r2, zres, "o", ms=4, color=C_M2, label="steady states, x and y set points")
    rr = np.linspace(0, r2.max(), 50)
    ax.plot(rr, kappa * rr, color=INK, lw=1.0, label=f"κ = {kappa * 1e3:.2f}e-3 /mm (M2 fit)")
    ax.set(xlabel="$X_x^2+X_y^2$ (mm²)", ylabel="z after removing linear part (mm)")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "task_space_lift.png")
    plt.close(fig)
    return float(coef[2])


# ---------------------------------------------------------------------------- report


def embed_figures(path: Path) -> int:
    """Replace src of every <img data-fig="name.png" ...> with a fresh data URI."""
    html = path.read_text()
    count = 0

    def sub(m):
        nonlocal count
        name = m.group(1)
        data = base64.b64encode((FIG_DIR / name).read_bytes()).decode()
        count += 1
        return f'<img data-fig="{name}" src="data:image/png;base64,{data}"'

    html = re.sub(r'<img data-fig="([^"]+)" src="[^"]*"', sub, html)
    path.write_text(html)
    return count


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--embed-only", action="store_true")
    args = ap.parse_args()
    if args.embed_only:
        print(f"embedded {embed_figures(REPORT)} figures in {REPORT}")
        return
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    style()

    print("running identification experiments ...")
    sp_runs = {axis: [run_experiment(setpoint_sequence(axis, lv), SEED_SETPOINT) for lv in SP_DESIGN[axis][1]]
               for axis in AX}
    bb = run_experiment(random_steps(150.0, 1.5, 7.5, (0.3, 3.0), seed=22), SEED_BROADBAND)
    reg = regulator_static(SEED_REGULATOR)

    act = fit_actuator(bb)
    m1 = identify_m1(sp_runs)
    m1_bb_a = fit_m1_broadband(m1, bb)
    mech1 = fit_mechanics(bb, order=1)
    mech2 = fit_mechanics(bb, order=2)
    mech2_s1 = fit_x_with_reservoir(bb)
    sines = sine_check(SEED_SINE, [0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 1.3, 1.6, 2.0, 2.5])

    print("running validation experiments ...")
    val_inputs = {
        "V1 multisine": multisine(60.0, 3.0, seed=31),
        "V2 open-loop circle": open_loop_circle(40.0),
        "V3 random steps": random_steps(90.0, 1.0, 8.0, (1.0, 6.0), seed=33),
    }
    results, recs, preds_all = {}, {}, {}
    for name, du in val_inputs.items():
        rec = run_experiment(du, SEED_VAL[name])
        recs[name] = rec
        p_hat = actuator(rec.du, act)
        preds = {
            "M1-LPV": simulate_m1(m1, rec, lpv=True),
            "M1-LTI": simulate_m1(m1, rec, lpv=False),
            "M2": simulate_mech(mech1, p_hat, 1),
            "M2 (no z lift)": simulate_mech(mech1, p_hat, 1, lift=False),
            "M3": simulate_mech(mech2, p_hat, 2),
            "M2 mechanics only (measured p in)": simulate_mech(mech1, rec.dp[:, 1:], 1),
            "M3 mechanics only (measured p in)": simulate_mech(mech2, rec.dp[:, 1:], 2),
        }
        preds_all[name] = preds
        results[name] = {k: rmse(v, rec.X) for k, v in preds.items()}
        xs1 = simulate_x_with_reservoir(mech2_s1, rec.dp)
        results[name]["M3 x with measured S1 (mechanics only), x only"] = rmse(xs1, rec.X[:, 0])
        results[name]["vs_noise_free_tip"] = {k: rmse(v, rec.X_clean) for k, v in preds.items()}
        results[name]["vs_noise_free_tip"]["M3 x with measured S1 (mechanics only), x only"] = rmse(
            xs1, rec.X_clean[:, 0])
        s1 = rec.p[:, 0]
        results[name]["reservoir_S1_drift_psi_per_min"] = float(60 * np.polyfit(rec.t, s1, 1)[0])
        results[name]["actuator_rmse_psi"] = rmse(p_hat, rec.dp[:, 1:])
        results[name]["reservoir_S1_range_psi"] = [float(rec.p[:, 0].min()), float(rec.p[:, 0].max())]
        results[name]["X_rms_mm"] = np.sqrt(np.mean(rec.X**2, 0)).tolist()

    # Disturbance seen by the per-axis first-order models on V1 (for NDOB bounds).
    rec = recs["V1 multisine"]
    G = np.array(m1["G"])
    a_m1 = np.array([m1["local"][a]["k_mean"] / m1["local"][a]["c_mean"] for a in AX])
    v_m1 = rec.du @ G.T / np.array([m1["local"][a]["k_mean"] for a in AX])
    dist = {"M1-LTI (command in)": disturbance_stats(rec.X, a_m1, v_m1)}
    a_m2 = np.array([mech1[a]["a"] for a in AX])
    v_m2 = np.stack([rec.dp[:, 1:] @ np.array(mech1[a]["H"]) for a in AX], 1)
    dist["M2 mechanics (measured p in)"] = disturbance_stats(rec.X, a_m2, v_m2)

    print("writing figures ...")
    fig_actuator(bb, act, reg)
    fig_scheduling(m1)
    fig_freq(sines, mech1, mech2)
    fig_validation("V1 multisine", recs["V1 multisine"], preds_all["V1 multisine"], "task_space_validation_v1.png")
    fig_validation("V2 open-loop circle", recs["V2 open-loop circle"], preds_all["V2 open-loop circle"],
                   "task_space_validation_v2.png")
    kappa_static = fig_lift(sp_runs, mech1["z"]["kappa_per_mm"])

    out = {
        "plant": "make_calibrated_sim(control_hz=100, seed=s)",
        "signals_used": ["time", "tip_pos (+ N(0, 0.1 mm) per axis, rng 1000+seed)", "segment_pressures"],
        "seeds": {"setpoint": SEED_SETPOINT, "broadband": SEED_BROADBAND, "sine_check": SEED_SINE,
                  "regulator_staircase": SEED_REGULATOR, "validation": SEED_VAL},
        "home": {"settle_s": SETTLE_S, "home_tip_mm_seed1": sp_runs["x"][2].home_mm.tolist(),
                 "home_pressure_psi_seed1": sp_runs["x"][2].p_home.tolist()},
        "actuator": act,
        "regulator_static": reg,
        "M1": m1,
        "M1_broadband_first_order_a": m1_bb_a,
        "M2_mechanics": mech1,
        "M3_mechanics": mech2,
        "M3_x_with_measured_S1": mech2_s1,
        "z_lift_kappa_static_per_mm": kappa_static,
        "sine_check": sines,
        "validation_rmse_mm": results,
        "disturbance_V1": dist,
    }
    (FIG_DIR / "task_space_model.json").write_text(json.dumps(out, indent=1))

    print("\nactuator:", {k: np.round(v, 4).tolist() for k, v in act.items()})
    print("regulator static gain/bias:", np.round(reg["affine_gain"], 4), np.round(reg["affine_bias"], 4))
    print("G (mm/psi):\n", np.round(G, 4))
    for axis in AX:
        L = m1["local"][axis]
        print(f"M1 {axis}: k={[round(p['k'], 4) for p in L['points']]} c={[round(p['c'], 4) for p in L['points']]} "
              f"rho={[round(p['rho'], 3) for p in L['points']]} a spread {L['a_spread_pct']:.1f}% "
              f"K spread {L['K_spread_pct']:.1f}%")
    print("M1 broadband a:", np.round(m1_bb_a, 4))
    print("M2 mech:", json.dumps(mech1, indent=None))
    print("M3 mech:", json.dumps(mech2, indent=None))
    print("kappa static regression:", kappa_static)
    print("sines:", [(s["f_hz"], round(abs(s["ratio_re"] + 1j * s["ratio_im"]), 3)) for s in sines])
    print("\nFree-run RMSE (mm) per axis x, y, z:")
    for name, r in results.items():
        print(f"  {name}")
        for k, v in r.items():
            v = np.atleast_1d(v) if not isinstance(v, dict) else v
            if isinstance(v, dict):
                for k2, v2 in v.items():
                    print(f"    clean: {k2:31s} {np.round(v2, 3).tolist()}")
            else:
                print(f"    {k:38s} {np.round(v, 3).tolist()}")
    print("disturbance:", json.dumps(dist))
    if REPORT.exists():
        print(f"embedded {embed_figures(REPORT)} figures in {REPORT}")


if __name__ == "__main__":
    main()
