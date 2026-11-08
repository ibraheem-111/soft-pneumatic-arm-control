"""Drive the simulator and three models with the same commands and compare the tip, per axis.

Ground truth: make_calibrated_sim(control_hz=100, seed=0), 5 s settle at 4.5 psi, then the
excitation; tip = obs["tip_pos"] minus the settled tip, in mm (the tip has no sensor noise).
Models, each simulated open loop (free run) from rest with the same commands u = cmd - 4.5 psi:

  one-mode  Nikoletta's reduced model (scripts/smc_reduced_model.reduced_model, notebook §7):
            y'' + c y' + k y = b dp per axis, corrected sign b = +|b|, no actuator dynamics,
            no z. Plane inputs dp_x = -u3 (S1 sealed, S3 alone), dp_y = u2 - u4.
  M2        scripts/task_space_model.py (docs/figures/task_space_model.json via ndosmc.TaskSpaceModel):
            tau p' = -p + g u(t - T) per segment, y' = -a y + a (H p) for x, y, z.
  F         scripts/sysid_frf.py (models/sysid_calibrated_v2.json): per-axis state space with the
            identified delay, channels nu_x = -u3, nu_y = u2 - u4 (the identification map), no z.
All are ZOH-discretised at 100 Hz; the model output after step k is compared with the simulator's
tip after step k (as in evaluate_controller).

Excitations (seeds differ from every identification and validation seed in the repo):
  random_steps  independent random levels per segment, uniform 1.5-7.5 psi, held 0.5-3 s, 60 s, seed 31
  chirp         S2-S4 = 4.5 + 2.5 sin(phase + 0, 120, 240 deg), frequency swept 0.02 -> 1 Hz
                (logarithmic) over 60 s

Metrics per axis, whole run: RMSE (mm) and fit = 100 (1 - |y_model - y| / |y - mean(y)|) %.
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
MODELS = ("one-mode", "M2", "F")
LABELS = {"truth": "MuJoCo simulator (truth)", "one-mode": "One-mode model (Nikoletta)",
          "M2": "M2 (task-space, identified)", "F": "F (frequency-identified)"}


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


def simulate_one_mode(u: np.ndarray) -> np.ndarray:
    k, c, b = S.reduced_model(include_gravity=False)
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


def simulate_f(u: np.ndarray) -> np.ndarray:
    axes = json.loads(F_MODEL.read_text())["axes"]
    nu = plane_inputs(u)
    out = np.full((len(u), 3), np.nan)
    for i, name in enumerate(("x", "y")):
        ax = axes[name]
        A, B, C = np.array(ax["A"]), np.array(ax["B"])[:, None], np.array(ax["C"])[None, :]
        out[:, i] = 1000.0 * _lsim(A, B, C, nu[:, [i]], delay=int(ax["delay_samples"]))[:, 0]
    return out


SIMULATE = {"one-mode": simulate_one_mode, "M2": simulate_m2, "F": simulate_f}


# ---------------------------------------------------------------------------- metrics, figure

def metrics(truth: np.ndarray, model: np.ndarray) -> dict:
    out = {}
    for i, ax in enumerate("xyz"):
        if np.isnan(model[:, i]).all():
            out[ax] = None
            continue
        err = model[:, i] - truth[:, i]
        out[ax] = dict(rmse_mm=float(np.sqrt(np.mean(err**2))),
                       fit_pct=float(100 * (1 - np.linalg.norm(err) / np.linalg.norm(truth[:, i] - truth[:, i].mean()))))
    return out


def figure(kind: str, t: np.ndarray, cmds: np.ndarray, truth: np.ndarray, sims: dict, path: Path) -> None:
    P.apply()
    fig, axs = P.figure(4, 1, sharex=True)
    for j, seg in enumerate(("S2", "S3", "S4")):
        axs[0, 0].plot(t, cmds[:, j], **P.line(seg, len(t)))
    axs[0, 0].set(ylabel="command (psi)", title=f"Excitation: {kind}")
    P.legend_outside(axs[0, 0])
    for i, ax_name in enumerate("xyz"):
        ax = axs[i + 1, 0]
        ax.plot(t, truth[:, i], **{**P.REFERENCE, "label": LABELS["truth"]})
        for m in MODELS:
            if not np.isnan(sims[m][:, i]).all():
                ax.plot(t, sims[m][:, i], **P.line(LABELS[m], len(t)))
        ax.set(ylabel=f"tip {ax_name} − settled (mm)", title=f"{ax_name} axis")
        P.legend_outside(ax)
    axs[-1, 0].set_xlabel("time (s)")
    for row in axs:
        row[0].grid(alpha=0.3)
    P.save(fig, path)


def run(kind: str, seconds: float) -> dict:
    cmds = excitation(kind, seconds)
    u = cmds - BASE
    truth = simulate_truth(cmds)
    sims = {m: SIMULATE[m](u) for m in MODELS}
    t = np.arange(len(cmds)) * DT
    figure(kind, t, cmds, truth, sims, FIG / f"model_comparison_{kind}.png")
    return dict(cmds=cmds, truth=truth, sims=sims, t=t,
                metrics={m: metrics(truth, sims[m]) for m in MODELS})


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
    (FIG / "model_comparison.json").write_text(json.dumps(out, indent=1))
    np.savez_compressed(FIG / "model_comparison_data.npz", **arrays)
    print("\nwrote docs/figures/model_comparison.json, model_comparison_data.npz, model_comparison_<excitation>.png")


if __name__ == "__main__":
    main()
