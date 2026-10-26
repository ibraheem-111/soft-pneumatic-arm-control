"""Tracking tests for the course's other reference motions on make_calibrated_sim.

Motions follow the RAS 556 project slides ("Simulation Results ... Motion"):
- triangular: straight-line (Cartesian) interpolation through three vertices,
  V0 on +x, V1 upper left, V2 lower left (equilateral, circumradius 7 mm), one lap
  per 10 s at constant speed, starting at V0;
- axial: the tip oscillates along one horizontal axis, 7 mm sin(2 pi 0.1 t), with
  the other axis held at the settled position (run along y and along x).
Amplitude, period, settle and duration match the package's CircularTrackingTask
(7 mm, 0.1 Hz, 5 s settle at 4.5 psi, 30 s scored), and the loop reproduces
evaluate_controller exactly (checked on the circle). Scoring: lateral (x-y) RMSE
over the whole run; for axial motion also the error on the held axis.

Usage:
    uv run python scripts/motion_tests.py      # about 3 min
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from soft_robotic_arm import make_calibrated_sim
from soft_robotic_arm.evaluation import CircularTrackingTask, make_reference

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mpc_controller as mpc  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
TASK = CircularTrackingTask()          # shared settings: rate, settle, duration, radius, frequency, baseline


def circle(home: np.ndarray, t: float) -> np.ndarray:
    return make_reference(home, t, TASK)


def triangle(home: np.ndarray, t: float) -> np.ndarray:
    angles = 2 * np.pi * np.arange(4) / 3
    verts = TASK.radius_m * np.column_stack([np.cos(angles), np.sin(angles)])   # V0, V1, V2, V0
    s = (TASK.frequency_hz * t % 1.0) * 3
    k = int(s)
    ref = home.copy()
    ref[:2] += verts[k] + (s - k) * (verts[k + 1] - verts[k])
    return ref


def axial(axis: int):
    def ref_fn(home: np.ndarray, t: float) -> np.ndarray:
        ref = home.copy()
        ref[axis] += TASK.radius_m * np.sin(2 * np.pi * TASK.frequency_hz * t)
        return ref
    return ref_fn


MOTIONS = {"circle (harness check)": circle, "triangle": triangle,
           "axial along y": axial(1), "axial along x": axial(0)}


def run(make_ctrl, ref_fn, seed: int = 0) -> dict:
    """Same sequence as evaluate_controller: settle at the baseline, take the settled
    tip as home, then track for TASK.track_s with clipped commands."""
    limit = mpc.COMMAND_LIMIT_PSI
    sim = make_calibrated_sim(control_hz=TASK.control_hz, seed=seed)
    try:
        baseline = TASK.baseline_command(limit)
        obs = sim.reset()
        for _ in range(TASK.settle_steps):
            obs = sim.step(baseline)
        home = np.asarray(obs["tip_pos"], dtype=float).copy()
        ctrl = make_ctrl(home, ref_fn)
        n = TASK.track_steps
        tips, refs, cmds, s1 = np.empty((n, 3)), np.empty((n, 3)), np.empty((n, 3)), np.empty(n)
        for k in range(n):
            ref = ref_fn(home, k / TASK.control_hz)
            cmd = np.clip(np.asarray(ctrl.compute(float(obs["time"]), obs, ref), dtype=float), 0.0, limit)
            obs = sim.step(cmd)
            tips[k], refs[k], cmds[k] = obs["tip_pos"], ref, cmd
            s1[k] = np.mean(obs["pouch_pressures"][0])     # sealed S1 reservoir, measured
    finally:
        sim.close()
    lat = np.linalg.norm(tips[:, :2] - refs[:, :2], axis=1) * 1000
    return {"rmse": float(np.sqrt(np.mean(lat ** 2))), "max": float(lat.max()), "lat": lat,
            "effort": float(np.sqrt(np.mean((cmds - baseline) ** 2))), "tips": tips, "refs": refs,
            "cmds": cmds, "s1": s1, "home": home, "err_xy": (tips[:, :2] - refs[:, :2]) * 1000,
            "dz": (tips[:, 2] - home[2]) * 1000}


def controllers() -> dict:
    base = mpc.baselines()
    return {
        "MPC": lambda home, ref_fn: mpc.DelayMPCController(),
        "MPC, AR preview only": lambda home, ref_fn: mpc.DelayMPCController(preview="ar"),
        "MPC, true future ref (diagnostic)": lambda home, ref_fn: mpc.DelayMPCController(
            preview="oracle", oracle=lambda n: ref_fn(home, n / TASK.control_hz)),
        "SMC": lambda home, ref_fn: base["SMC lam=5 K=0.35 phi=0.02, deviation"](),
        "PD": lambda home, ref_fn: base["PD kp=500 kd=20"](),
    }


def main() -> None:
    ctrls = controllers()
    print("make_calibrated_sim, seed 0; 7 mm, 0.1 Hz, 30 s; lateral RMSE / max [mm], effort [psi]")
    results = {}
    for motion, ref_fn in MOTIONS.items():
        print(f"  {motion}")
        for name, make in ctrls.items():
            r = run(make, ref_fn)
            results[(motion, name)] = r
            extra = ""
            if motion.startswith("axial"):
                held = 0 if motion.endswith("y") else 1
                tail = slice(int(2 * TASK.control_hz), None)
                extra = (f"; held axis RMSE {np.sqrt(np.mean(r['err_xy'][:, held] ** 2)):.3f} mm; "
                         f"after 2 s {np.sqrt(np.mean(r['lat'][tail] ** 2)):.3f} mm; "
                         f"z by-product {r['dz'].min():+.2f} to {r['dz'].max():+.2f} mm")
            elif motion == "triangle":
                tail = slice(int(2 * TASK.control_hz), None)
                extra = f"; after 2 s {np.sqrt(np.mean(r['lat'][tail] ** 2)):.3f} mm"
            print(f"    {name:<36}{r['rmse']:7.3f} {r['max']:6.2f} {r['effort']:6.2f}{extra}")
    harness = mpc.evaluate(lambda: mpc.DelayMPCController()).rmse_mm
    print(f"  check: harness MPC on the circle {harness:.6f} mm vs this loop "
          f"{results[('circle (harness check)', 'MPC')]['rmse']:.6f} mm")

    for motion in MOTIONS:
        out = ROOT / "docs" / "figures" / f"motion_{motion.split(' (')[0].replace(' ', '_')}.png"
        motion_figure(motion, results, out)
        print(f"  figure: {out.relative_to(ROOT)}")


def motion_figure(motion: str, results: dict, out: Path) -> None:
    """Slide-style panels: tip trajectory, trajectory against z, tip deflection over time,
    pressure per segment over time (MPC), plus the error of every controller."""
    colors = {"MPC": "C0", "SMC": "C1", "PD": "C2"}
    t = np.arange(TASK.track_steps) / TASK.control_hz
    r0 = results[(motion, "MPC")]
    rel = lambda r: (r["tips"] - r["home"]) * 1000                       # noqa: E731
    ref = (r0["refs"] - r0["home"]) * 1000
    fig, ax = plt.subplots(2, 3, figsize=(16, 9))

    a = ax[0, 0]
    a.plot(ref[:, 0], ref[:, 1], "k--", lw=1, label="reference")
    for name, col in colors.items():
        p = rel(results[(motion, name)])
        a.plot(p[:, 0], p[:, 1], col, lw=1, label=f"{name} ({results[(motion, name)]['rmse']:.2f} mm)")
    if motion == "triangle":
        for k in range(3):
            v = 1000 * TASK.radius_m * np.array([np.cos(2 * np.pi * k / 3), np.sin(2 * np.pi * k / 3)])
            a.plot(*v, "o", color="C3", ms=7)
            a.annotate(f"V{k}", v, textcoords="offset points", xytext=(6, 6))
    a.set(title="Tip trajectory (x–y)", xlabel="tip x [mm]", ylabel="tip y [mm]", aspect="equal",
          xlim=(-9, 9), ylim=(-9, 9))
    a.legend(fontsize=7, loc="upper right")

    a = ax[0, 1]
    j = 0 if motion.endswith(" x") else 1
    p = rel(r0)
    a.plot(p[:, j], p[:, 2], "C0", lw=1)
    a.set(title=f"Tip trajectory ({'xy'[j]}–z plane), MPC", xlabel=f"tip {'xy'[j]} [mm]", ylabel="tip z change [mm]")

    a = ax[0, 2]
    for c, lab in enumerate(("tip x", "tip y", "tip z (rel)")):
        a.plot(t, p[:, c], f"C{c + 1}" if c < 2 else "0.4", lw=1, ls="--" if c == 2 else "-", label=lab)
    a.plot(t, ref[:, 0], "C1", lw=0.6, alpha=0.5)
    a.plot(t, ref[:, 1], "C2", lw=0.6, alpha=0.5)
    a.set(title="Tip deflection, MPC (faint: reference)", xlabel="time [s]", ylabel="tip deflection [mm]")
    a.legend(fontsize=8)

    a = ax[1, 0]
    a.plot(t, r0["s1"], color="C1", lw=1, label="Seg 1 (sealed, measured)")
    for c, (lab, col) in enumerate((("Seg 2", "C2"), ("Seg 3", "C3"), ("Seg 4", "C0"))):
        a.plot(t, r0["cmds"][:, c], col, lw=1, label=lab)
    a.set(title="Pressure commands, MPC", xlabel="time [s]", ylabel="pressure [psi]", ylim=(-0.3, 9.3))
    a.legend(fontsize=8, ncol=2)

    a = ax[1, 1]
    for name, col in colors.items():
        a.semilogy(t, np.maximum(results[(motion, name)]["lat"], 1e-3), col, lw=1, label=name)
    a.set(title="Lateral tracking error", xlabel="time [s]", ylabel="error [mm]", ylim=(1e-2, 10))
    a.legend(fontsize=8)

    a = ax[1, 2]
    a.axis("off")
    rows = [(n, results[(motion, n)]) for n in ("MPC", "MPC, AR preview only", "MPC, true future ref (diagnostic)",
                                               "SMC", "PD")]
    a.table(cellText=[[n, f"{r['rmse']:.3f}", f"{np.sqrt(np.mean(r['lat'][200:] ** 2)):.3f}", f"{r['effort']:.2f}"]
                      for n, r in rows],
            colLabels=["controller", "RMSE [mm]", "after 2 s [mm]", "effort [psi]"], loc="center",
            cellLoc="left", colWidths=[0.52, 0.16, 0.18, 0.16]).scale(1, 1.6)
    for a in ax.flat[:5]:
        a.grid(True, which="both", alpha=0.3)
    fig.suptitle(f"{motion.capitalize()}: make_calibrated_sim, 7 mm, 0.1 Hz, seed 0")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
