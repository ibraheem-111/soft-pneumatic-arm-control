"""Drive the simulator and three models with the same commands and compare the tip, per axis.

Ground truth: make_calibrated_sim(control_hz=100, seed=0), 5 s settle at 4.5 psi, then the
excitation; tip = obs["tip_pos"] minus the settled tip, in mm (the tip has no sensor noise).
Models, each simulated open loop (free run) from rest with the same commands u = cmd - 4.5 psi:

  one-mode    Nikoletta's reduced model, as in her latest notebook (Copy_of_soft_robotic_arm_
              SMC_Smith-2.ipynb §7, same as scripts/smc_reduced_model.reduced_model):
              y'' + c y' + k y = b dp per axis, k = 10.60, c = 9.231, corrected sign b = +|b|,
              no actuator dynamics, no z. Plane inputs dp_x = -u3 (S1 sealed), dp_y = u2 - u4.
  one-mode+g  the same derivation with the hanging chain's gravity stiffness added
              (reduced_model(include_gravity=True): k = 51.45, c = 9.409); from the review of
              her model, not in her notebooks.
  M2          scripts/task_space_model.py (docs/figures/task_space_model.json via ndosmc.TaskSpaceModel):
              tau p' = -p + g u(t - T) per segment, y' = -a y + a (H p) for x, y, z.
  M3          M2's actuator with second-order mechanics (task_space_model.json M3_mechanics):
              y'' + 2 zeta w y' + w^2 y = w^2 (H p) per axis.
  F         scripts/sysid_frf.py (models/sysid_calibrated_v2.json): per-axis state space with the
            identified delay, channels nu_x = -u3, nu_y = u2 - u4 (the identification map), no z.
All are ZOH-discretised at 100 Hz; the model output after step k is compared with the simulator's
tip after step k (as in evaluate_controller).

Excitations (seeds differ from every identification and validation seed in the repo):
  random_steps  independent random levels per segment, uniform 1.5-7.5 psi, held 0.5-3 s, 60 s, seed 31
  chirp         S2-S4 = 4.5 + 2.5 sin(phase + 0, 120, 240 deg), frequency swept 0.02 -> 1 Hz
                (logarithmic) over 60 s

M2 + NDO is not a free-run model: a disturbance observer needs the measured tip, so with no
measurement it adds nothing. It is scored on what the delay-aware controllers use the model for,
predicting the tip one dead time T ahead from the tip measured up to now:
  hold      y_hat(t+T) = y(t)
  M2        y_hat(t+T) = y(t) + x_m(t) - x_m(t-T)   (Smith predictor; x_m = M2 without the dead time)
  M2 + NDO  M2 + d_hat (1 - exp(-a T)) / a           (observer estimate of the unmodelled rate, held
            over T; ndosmc.NDOB with l2 = 0: d_hat = z + l1 y, z' = l1 (a y - a H p_hat - d_hat))

The actuator identification (identify_actuator) steps one input by +1 psi and fits dead time, lag
and gain to the measured pouch pressure of that column, the way it would be done without
knowing them.

Metrics per axis, whole run: RMSE, max |error| and bias = mean error (mm), and
fit = 100 (1 - |y_model - y| / |y - mean(y)|) %. Static gains (mm/psi): the tip change 10 s after
a +1 psi step on S3 (x) and on S2 (y) from the 4.5 psi baseline, for the simulator and each model.
Writes docs/figures/model_comparison.json (metrics, settings), model_comparison_data.npz (time
series) and model_comparison_<excitation>.png.

Usage:
    uv run python scripts/compare_models_excitation.py [--excitation random_steps chirp] [--seconds 60]
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import cont2discrete

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ndosmc as N  # noqa: E402
import plot_style as P  # noqa: E402
import smc_reduced_model as S  # noqa: E402
from soft_robotic_arm import make_calibrated_sim  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
FIG = ROOT / "docs" / "figures"
F_MODEL = ROOT / "models" / "sysid_calibrated_v2.json"
HZ, DT, BASE, SETTLE_S = 100.0, 0.01, 4.5, 5.0
MODELS = ("one-mode", "one-mode+g", "M2", "M3", "F")
LABELS = {"truth": "Simulator", "one-mode": "One-mode", "one-mode+g": "One-mode + gravity",
          "M2": "M2", "M3": "M3", "F": "F"}
SLUG = {"one-mode": "onemode", "one-mode+g": "onemode_gravity", "M2": "M2", "M3": "M3", "F": "F"}
SIM_STYLE = dict(color=P.BLACK, ls="-", lw=3.2, alpha=0.85, label="Simulator")


# ---------------------------------------------------------------------------- excitations

def excitation(kind: str, seconds: float, seed: int = 31) -> np.ndarray:
    """Absolute commands (n, 3) for S2, S3, S4 in psi."""
    n = int(round(seconds * HZ))
    if kind == "random_steps":
        g = np.random.default_rng(seed)
        out = np.empty((n, 3))
        for j in range(3):
            k = 0
            while k < n:
                hold = int(round(g.uniform(0.5, 3.0) * HZ))
                out[k:k + hold, j] = g.uniform(1.5, 7.5)
                k += hold
        return out
    if kind == "chirp":
        t = np.arange(n) * DT
        f0, f1 = 0.02, 1.0
        phase = 2 * np.pi * f0 * seconds / np.log(f1 / f0) * ((f1 / f0) ** (t / seconds) - 1)
        return BASE + 2.5 * np.sin(phase[:, None] + np.radians([0.0, 120.0, 240.0]))
    raise ValueError(kind)


# ---------------------------------------------------------------------------- simulator

def simulate_truth(cmds: np.ndarray, seed: int = 0) -> np.ndarray:
    """Tip after each command step minus the settled tip, mm, (n, 3)."""
    sim = make_calibrated_sim(control_hz=HZ, seed=seed)
    obs = sim.reset()
    for _ in range(int(round(SETTLE_S * HZ))):
        obs = sim.step(np.full(3, BASE))
    home = np.asarray(obs["tip_pos"], float).copy()
    out = np.empty((len(cmds), 3))
    for k, c in enumerate(cmds):
        obs = sim.step(c)
        out[k] = 1000.0 * (np.asarray(obs["tip_pos"], float) - home)
    sim.close()
    return out


# ---------------------------------------------------------------------------- models

def _lsim(A, B, C, u: np.ndarray, delay: int = 0) -> np.ndarray:
    """ZOH-discretised x' = A x + B u(t - delay*DT), y = C x, from rest; y after each step."""
    Ad, Bd, *_ = cont2discrete((A, B, C, np.zeros((C.shape[0], B.shape[1]))), DT)
    ud = np.vstack([np.zeros((delay, u.shape[1])), u])[: len(u)]
    x = np.zeros(A.shape[0])
    y = np.empty((len(u), C.shape[0]))
    for k in range(len(u)):
        x = Ad @ x + Bd @ ud[k]
        y[k] = C @ x
    return y


def plane_inputs(u: np.ndarray) -> np.ndarray:
    """(n, 2) plane inputs (x, y) from segment deviations u = (u2, u3, u4): (-u3, u2 - u4)."""
    return np.column_stack([-u[:, 1], u[:, 0] - u[:, 2]])


def simulate_one_mode(u: np.ndarray, gravity: bool = False) -> np.ndarray:
    k, c, b = S.reduced_model(include_gravity=gravity)
    b = abs(b)                                   # corrected sign (notebook §13)
    A, B, C = np.array([[0.0, 1.0], [-k, -c]]), np.array([[0.0], [b]]), np.array([[1.0, 0.0]])
    dp = plane_inputs(u)
    out = np.full((len(u), 3), np.nan)
    for i in (0, 1):
        out[:, i] = 1000.0 * _lsim(A, B, C, dp[:, [i]])[:, 0]
    return out


def simulate_m2(u: np.ndarray) -> np.ndarray:
    m = N.TaskSpaceModel.load()
    act, H = m.actuator, m.B_measured
    a = np.array([ax.params(0.0)[0] for ax in m.axes_measured])
    tau, g = act.tau_s, act.gain
    A = np.block([[-np.diag(a), np.diag(a) @ H], [np.zeros((3, 3)), -np.eye(3) / tau]])
    B = np.vstack([np.zeros((3, 3)), np.diag(g) / tau])
    C = np.hstack([np.eye(3), np.zeros((3, 3))])
    return _lsim(A, B, C, u, delay=int(round(act.delay_s * HZ)))


def simulate_m3(u: np.ndarray) -> np.ndarray:
    m = N.TaskSpaceModel.load()
    act = m.actuator
    mech = json.loads(N.MODEL_JSON.read_text())["M3_mechanics"]
    w = np.array([mech[a]["wn"] for a in "xyz"])
    z = np.array([mech[a]["zeta"] for a in "xyz"])
    H = np.array([mech[a]["H"] for a in "xyz"])
    tau, g, O = act.tau_s, act.gain, np.zeros((3, 3))
    A = np.block([[O, np.eye(3), O],
                  [-np.diag(w**2), -np.diag(2 * z * w), np.diag(w**2) @ H],
                  [O, O, -np.eye(3) / tau]])
    B = np.vstack([O, O, np.diag(g) / tau])
    C = np.hstack([np.eye(3), O, O])
    return _lsim(A, B, C, u, delay=int(round(act.delay_s * HZ)))


def simulate_f(u: np.ndarray) -> np.ndarray:
    axes = json.loads(F_MODEL.read_text())["axes"]
    nu = plane_inputs(u)
    out = np.full((len(u), 3), np.nan)
    for i, name in enumerate(("x", "y")):
        ax = axes[name]
        A, B, C = np.array(ax["A"]), np.array(ax["B"])[:, None], np.array(ax["C"])[None, :]
        out[:, i] = 1000.0 * _lsim(A, B, C, nu[:, [i]], delay=int(ax["delay_samples"]))[:, 0]
    return out


SIMULATE = {"one-mode": simulate_one_mode, "one-mode+g": lambda u: simulate_one_mode(u, gravity=True),
            "M2": simulate_m2, "M3": simulate_m3, "F": simulate_f}


def m2_parts():
    """M2's mechanics rate a (1/s), pressure map H (mm/psi), lag tau (s), gain g, dead time (samples)."""
    m = N.TaskSpaceModel.load()
    a = np.array([ax.params(0.0)[0] for ax in m.axes_measured])
    return a, m.B_measured, m.actuator.tau_s, m.actuator.gain, int(round(m.actuator.delay_s * HZ))


PREDICTORS = ("hold", "M2", "M2 + NDO")
NDO_L1 = (0.1, 1.0)            # 0.1 1/s: the gain of the delay-aware NDOSMC (Nikoletta's notebook)


def predict_ahead(truth: np.ndarray, u: np.ndarray, l1: float = 0.1) -> dict:
    """Tip predicted one dead time T ahead from the tip measured up to t (module docstring).
    Returns name -> (n - T, 3) predictions of truth[T:], plus "d_hat" (n, 3), mm/s."""
    a, H, tau, g, nd = m2_parts()
    A = np.block([[-np.diag(a), np.diag(a) @ H], [np.zeros((3, 3)), -np.eye(3) / tau]])
    B = np.vstack([np.zeros((3, 3)), np.diag(g) / tau])
    xp = _lsim(A, B, np.eye(6), u)               # M2 with the dead time removed: [x_m, p_m]
    x_m, p_m = xp[:, :3], xp[:, 3:]
    n = len(truth)
    x_back = np.vstack([np.zeros((nd, 3)), x_m])[:n]          # x_m(t - T)
    p_now = np.vstack([np.zeros((nd, 3)), p_m])[:n]           # model pressure acting now: p_m(t - T)
    d_hat, z = np.zeros((n, 3)), -l1 * truth[0]
    for k in range(n):
        d_hat[k] = z + l1 * truth[k]
        z = z + DT * l1 * (a * truth[k] - a * (H @ p_now[k]) - d_hat[k])
    smith = truth + x_m - x_back
    T = nd * DT
    return {"hold": truth[:n - nd], "M2": smith[:n - nd],
            "M2 + NDO": (smith + d_hat * (1 - np.exp(-a * T)) / a)[:n - nd], "d_hat": d_hat}


def prediction_metrics(truth: np.ndarray, u: np.ndarray) -> dict:
    """RMSE (mm) of each T-ahead predictor per axis, NDO at each gain in NDO_L1."""
    nd = m2_parts()[4]
    target = truth[nd:]
    rm = lambda p: {ax: float(np.sqrt(np.mean((p[:, i] - target[:, i]) ** 2))) for i, ax in enumerate("xyz")}  # noqa: E731
    pr = predict_ahead(truth, u, NDO_L1[0])
    out = {"hold": rm(pr["hold"]), "M2": rm(pr["M2"])}
    for l1 in NDO_L1:
        out[f"M2 + NDO (l1 = {l1:g})"] = rm(predict_ahead(truth, u, l1)["M2 + NDO"])
    return out


def identify_actuator(seg: int = 1, seconds: float = 6.0, pre_s: float = 2.0) -> dict:
    """+1 psi step on one input (0 = S2, 1 = S3, 2 = S4) after settling at 4.5 psi; measured pouch
    pressure of that column (mean of its 5 pouches). Dead time = first sample outside the pre-step
    band (mean +- 5 std); then least squares on p = p0 + G (1 - exp(-(t - T) / tau)) for T, tau, G."""
    from scipy.optimize import curve_fit
    sim = make_calibrated_sim(control_hz=HZ, seed=0)
    sim.reset()
    col = None
    p = []
    n_pre = int(round(pre_s * HZ))
    for k in range(int(round((SETTLE_S + pre_s + seconds) * HZ))):
        c = np.full(3, BASE)
        if k >= int(round((SETTLE_S + pre_s) * HZ)):
            c[seg] += 1.0
        obs = sim.step(c)
        col = int(np.asarray(obs["actuator_columns"])[seg])
        p.append(float(np.mean(obs["pouch_pressures"][col])))
    sim.close()
    p = np.array(p[-(n_pre + int(round(seconds * HZ))):])
    t = (np.arange(len(p)) - n_pre) * DT                     # 0 = first sample with the new command
    p0, sd = p[:n_pre].mean(), p[:n_pre].std()
    k_first = int(np.argmax(np.abs(p[n_pre:] - p0) > 5 * sd))
    model = lambda tt, T, tau, G: p0 + G * (1 - np.exp(-np.clip(tt - T, 0, None) / tau))  # noqa: E731
    (T, tau, G), _ = curve_fit(model, t[n_pre:], p[n_pre:], p0=(k_first * DT, 0.5, 1.0))
    return dict(t=t, p=p, fit=model(t, T, tau, G), p0=float(p0), noise_std=float(sd),
                dead_time_first_rise_s=k_first * DT, dead_time_s=float(T), tau_s=float(tau), gain=float(G))


def step_commands(seg: int, seconds: float = 10.0) -> np.ndarray:
    """+1 psi on one segment (0 = S2, 1 = S3, 2 = S4) from the 4.5 psi baseline."""
    c = np.full((int(round(seconds * HZ)), 3), BASE)
    c[:, seg] += 1.0
    return c


def static_gains() -> dict:
    """Tip change 10 s after +1 psi on S3 (x) and on S2 (y), mm/psi, simulator and models."""
    out = {}
    for name, seg, axis in (("x per psi on S3", 1, 0), ("y per psi on S2", 0, 1)):
        cmds = step_commands(seg)
        out[name] = {"truth": float(simulate_truth(cmds)[-1, axis])}
        for m in MODELS:
            out[name][m] = float(SIMULATE[m](cmds - BASE)[-1, axis])
    return out


# ---------------------------------------------------------------------------- metrics, figure

def metrics(truth: np.ndarray, model: np.ndarray) -> dict:
    out = {}
    for i, ax in enumerate("xyz"):
        if np.isnan(model[:, i]).all():
            out[ax] = None
            continue
        err = model[:, i] - truth[:, i]
        out[ax] = dict(rmse_mm=float(np.sqrt(np.mean(err**2))), max_abs_mm=float(np.abs(err).max()),
                       bias_mm=float(err.mean()),
                       fit_pct=float(100 * (1 - np.linalg.norm(err) / np.linalg.norm(truth[:, i] - truth[:, i].mean()))))
    return out


def _axis_panels(axs, t, truth, sims: dict, names) -> None:
    for i, ax_name in enumerate("xyz"):
        ax = axs[i]
        ax.plot(t, truth[:, i], **SIM_STYLE)
        for m in names:
            if not np.isnan(sims[m][:, i]).all():
                ax.plot(t, sims[m][:, i], **P.line(LABELS[m], len(t)))
        ax.set(ylabel=f"tip {ax_name} (mm)")
        ax.grid(alpha=0.3)


def figure_overview(kind: str, t, cmds, truth, sims: dict, path: Path | None = None):
    """Commands, then x, y, z with the simulator and every model on the same axes."""
    P.apply()
    fig, axs = P.figure(4, 1, sharex=True, layout="constrained")
    for j, seg in enumerate(("S2", "S3", "S4")):
        axs[0, 0].plot(t, cmds[:, j], **P.line(seg, len(t)))
    axs[0, 0].set(ylabel="command (psi)", title=f"{kind}: commands and tip, all models")
    axs[0, 0].grid(alpha=0.3)
    P.legend_outside(axs[0, 0])
    _axis_panels(axs[1:, 0], t, truth, sims, MODELS)
    P.legend_outside(axs[1, 0])
    axs[-1, 0].set_xlabel("time (s)")
    if path:
        P.save(fig, path)
    return fig


def figure_model(kind: str, m: str, t, truth, sims: dict, met: dict, path: Path | None = None):
    """x, y, z: the simulator and one model, RMSE in each title."""
    P.apply()
    fig, axs = P.figure(3, 1, sharex=True, layout="constrained")
    _axis_panels(axs[:, 0], t, truth, sims, [m])
    for i, a in enumerate("xyz"):
        v = met[m][a]
        axs[i, 0].set_title(f"{LABELS[m]} vs simulator, {a}: " + (f"RMSE {v['rmse_mm']:.3f} mm, fit {v['fit_pct']:.1f} %"
                                                                 if v else "this model has no z output"))
    axs[0, 0].legend(loc="upper right")
    axs[-1, 0].set_xlabel("time (s)")
    if path:
        P.save(fig, path)
    return fig


def figure_rmse(results: dict, path: Path | None = None):
    """Free-run RMSE per model and axis, one panel per excitation, value on every bar."""
    P.apply()
    fig, axs = P.figure(1, len(results), layout="constrained")
    w = 0.16
    for col, (kind, met) in enumerate(results.items()):
        ax = axs[0, col]
        for k, m in enumerate(MODELS):
            c = P.identity(LABELS[m])[0]
            for i, a in enumerate("xyz"):
                v = met[m][a]
                if v is None:
                    continue
                x = i + (k - 2) * w
                ax.bar(x, v["rmse_mm"], w, color=c, edgecolor="black", lw=0.6, label=LABELS[m] if i == 0 else None)
                ax.text(x, v["rmse_mm"] * 1.12, f"{v['rmse_mm']:.2f}", ha="center", va="bottom", fontsize=9, rotation=90)
        ax.set_yscale("log")
        ax.set_ylim(0.02, 80)
        ax.set_xticks(range(3), ["x", "y", "z"])
        ax.set(title=kind, ylabel="free-run RMSE (mm, log scale)")
        ax.grid(alpha=0.3, axis="y")
    axs[0, 0].legend(loc="upper right", fontsize=10)
    if path:
        P.save(fig, path)
    return fig


def figure_actuator(act: dict, path: Path | None = None):
    P.apply()
    fig, axs = P.figure(1, 1, layout="constrained")
    ax = axs[0, 0]
    ax.plot(act["t"], act["p"], **{**SIM_STYLE, "label": "measured pouch pressure, S3 column"})
    ax.plot(act["t"], act["fit"], color=P.ORANGE, ls=P.DASHED, lw=P.LW, label="fit: p0 + G (1 − e^(−(t−T)/τ))")
    ax.axvline(0, color="black", lw=1)
    ax.axvline(act["dead_time_s"], color="black", lw=1, ls=":")
    ax.text(0.03, act["p0"] + 0.04, "command +1 psi", fontsize=11)
    ax.text(act["dead_time_s"] + 0.05, act["p0"] + 0.5,
            f"T = {act['dead_time_s']:.2f} s\nτ = {act['tau_s']:.2f} s\nG = {act['gain']:.2f}", fontsize=11)
    ax.set(xlabel="time since the command step (s)", ylabel="pressure (psi)", xlim=(-1, 6),
           title="Actuator step response")
    ax.grid(alpha=0.3)
    ax.legend(loc="lower right")
    if path:
        P.save(fig, path)
    return fig


def figure_prediction(t, truth, u, path: Path | None = None, l1: float = 0.1):
    """Left: error of the 0.5 s-ahead prediction, M2 vs M2 + NDO. Right: the observer's d_hat."""
    P.apply()
    pred = predict_ahead(truth, u, l1)
    nd = m2_parts()[4]
    tt, target = t[nd:], truth[nd:]
    fig, axs = P.figure(3, 2, sharex=True, layout="constrained")
    fig.set_size_inches(P.WIDTH * 1.5, P.PANEL_H * 3)
    st = {"M2": dict(color=P.BLUE, ls="-", lw=P.LW, label="M2 (Smith prediction)"),
          "M2 + NDO": dict(color=P.ORANGE, ls=P.DASHED, lw=P.LW, label=f"M2 + NDO, l1 = {l1:g}")}
    for i, a in enumerate("xyz"):
        for k, kw in st.items():
            axs[i, 0].plot(tt, pred[k][:, i] - target[:, i], **kw)
        axs[i, 0].axhline(0, color="black", lw=1)
        axs[i, 0].set(ylabel=f"{a} error (mm)", title=f"{a}: prediction 0.5 s ahead − simulator")
        axs[i, 1].plot(t, pred["d_hat"][:, i], color=P.ORANGE, ls=P.DASHED, lw=P.LW, label="d̂")
        axs[i, 1].axhline(0, color="black", lw=1)
        axs[i, 1].set(ylabel=f"d̂ {a} (mm/s)", title=f"{a}: observer estimate d̂")
        for ax in axs[i]:
            ax.grid(alpha=0.3)
    axs[0, 0].legend(loc="upper right", fontsize=10)
    for ax in axs[-1]:
        ax.set_xlabel("time (s)")
    if path:
        P.save(fig, path)
    return fig


def run(kind: str, seconds: float) -> dict:
    cmds = excitation(kind, seconds)
    u = cmds - BASE
    truth = simulate_truth(cmds)
    sims = {m: SIMULATE[m](u) for m in MODELS}
    t = np.arange(len(cmds)) * DT
    met = {m: metrics(truth, sims[m]) for m in MODELS}
    met["prediction_one_dead_time_ahead"] = prediction_metrics(truth, u)
    figure_overview(kind, t, cmds, truth, sims, FIG / f"model_comparison_{kind}.png")
    for m in MODELS:
        figure_model(kind, m, t, truth, sims, met, FIG / f"model_comparison_{kind}_{SLUG[m]}.png")
    if kind == "random_steps":
        figure_prediction(t, truth, u, FIG / "model_prediction_random_steps.png")
    return dict(cmds=cmds, truth=truth, sims=sims, t=t, metrics=met)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--excitation", nargs="+", default=["random_steps", "chirp"])
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()
    out, arrays = dict(sim="make_calibrated_sim(control_hz=100, seed=0)", settle_s=SETTLE_S,
                       seconds=args.seconds, labels=LABELS, results={}), {}
    for kind in args.excitation:
        r = run(kind, args.seconds)
        out["results"][kind] = r["metrics"]
        arrays[f"{kind}_t"], arrays[f"{kind}_cmds"], arrays[f"{kind}_truth"] = r["t"], r["cmds"], r["truth"]
        for m in MODELS:
            arrays[f"{kind}_{m}"] = r["sims"][m]
        print(f"\n{kind}: per-axis RMSE mm (fit %)")
        for m in MODELS:
            cells = [f"{a}: {v['rmse_mm']:.3f} ({v['fit_pct']:.1f} %)" if v else f"{a}: no z output"
                     for a, v in r["metrics"][m].items()]
            print(f"  {LABELS[m]:<30}" + "   ".join(cells))
        print(f"  predicting the tip one dead time ahead from the measured tip, RMSE mm:")
        for name, v in r["metrics"]["prediction_one_dead_time_ahead"].items():
            print(f"    {name:<26}" + "   ".join(f"{a}: {e:.3f}" for a, e in v.items()))
    figure_rmse(out["results"], FIG / "model_comparison_rmse.png")
    act = identify_actuator()
    figure_actuator(act, FIG / "model_actuator_step.png")
    out["actuator_identified_from_step"] = {k: v for k, v in act.items() if not isinstance(v, np.ndarray)}
    print("\nactuator from a +1 psi step on S3:", out["actuator_identified_from_step"])
    out["static_gains_mm_per_psi"] = static_gains()
    print("\nstatic gains, mm/psi:", json.dumps(out["static_gains_mm_per_psi"], indent=1))
    (FIG / "model_comparison.json").write_text(json.dumps(out, indent=1))
    np.savez_compressed(FIG / "model_comparison_data.npz", **arrays)
    print("\nwrote docs/figures/model_comparison.json, model_comparison_data.npz, model_comparison_<excitation>.png")


if __name__ == "__main__":
    main()
