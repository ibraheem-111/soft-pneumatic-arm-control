"""Delay-compensated model-predictive tip controller for make_calibrated_sim.

Built on the per-axis models identified by `scripts/sysid_frf.py fit --plant
calibrated` (models/sysid_calibrated_v2.json): input = channel command u [psi],
output = tip position [m], x_{n+1} = A_d x_n + B_d u_{n-d}, with d = 50 samples
(0.5 s transport delay) and the first state equal to the measured channel pressure.

Per axis, every control step:
  1. Kalman filter: correct the state estimate [x; tip offset] with the measured tip
     and the measured channel pressure (pouch sensors).
  2. Delay predictor (Smith-predictor style): x_{n+d} = A_d^d x_n + sum A_d^{d-1-i} B_d u_{n-d+i},
     using the commands already sent but not yet acting.
  3. MPC without constraints: minimise sum_i (C x_{n+d+1+i} + offset - r(n+d+i))^2
     + rho sum_i (u_{n+i} - u_{n+i-1})^2 over a 3 s horizon; apply the first move.
     The future reference r(n+d+i) is predicted from the reference's own history:
     AR(2) with intercept on samples 0.1 s apart (exact for a sinusoid plus offset)
     until the reference is seen to repeat, then the detected period is replayed
     (exact for any periodic reference). Nothing about the trajectory is assumed.
  4. Channels to segments: S2 = b + uy/2, S3 = b - ux, S4 = b - uy/2 (the same map the
     model was identified with), clipped to 0-9 psi.

The controller is softarm.controllers.KalmanDelayMPC (softarm/controllers/kalman_mpc.py, reference
predictor in softarm/controllers/preview.py); this script evaluates it. Controller interface: the
package harness's compute(t, obs, ref) -> (3,) for S2-S4. Scored with
soft_robotic_arm.evaluation.evaluate_controller (CircularTrackingTask on make_calibrated_sim,
seed 0), against the PD and SMC baselines of softarm.controllers.presets.mpc_baselines
(the notebook model, corrected allocator).

Usage:
    uv run python scripts/mpc_controller.py            # evaluate and compare (about 2 min)
    uv run python scripts/mpc_controller.py --tune     # sweep rho on a separate tuning task
"""

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from soft_robotic_arm.evaluation import CircularTrackingTask

from softarm.constants import FIG, ROOT
from softarm.controllers import KalmanDelayMPC, NoisyTip
from softarm.controllers.kalman_mpc import TUNE_SEED, TUNE_TASK
from softarm.controllers.presets import mpc_baselines
from softarm.evaluation import evaluate, oracle_for


def row(name: str, r) -> str:
    return f"  {name:<40}{r.rmse_mm:>9.3f}{r.max_error_mm:>9.2f}{r.phase_lag_s:>9.2f}{r.effort_psi:>9.2f}"


def tune() -> None:
    task = CircularTrackingTask(**TUNE_TASK)
    print(f"Tuning rho on a separate task (radius {task.radius_m * 1000:g} mm, {task.frequency_hz} Hz, "
          f"sim seed {TUNE_SEED})")
    for rho in (1e-8, 1e-7, 1e-6, 1e-5, 1e-4):
        r = evaluate(KalmanDelayMPC(rho=rho), task, TUNE_SEED)
        print(f"  rho {rho:.0e}: RMSE {r.rmse_mm:.3f} mm, max {r.max_error_mm:.2f} mm, effort {r.effort_psi:.2f} psi")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tune", action="store_true", help="sweep rho on the tuning task")
    args = parser.parse_args()
    if args.tune:
        tune()
        return

    task = CircularTrackingTask()
    print("Scored task: make_calibrated_sim seed 0, 7 mm circle at 0.1 Hz, 30 s (package harness)")
    print(f"  {'controller':<40}{'RMSE mm':>9}{'max mm':>9}{'lag s':>9}{'effort':>9}")
    oracle = oracle_for(task, 0)
    ctrls = {"MPC, predicted reference": lambda: KalmanDelayMPC(),
             "MPC, AR preview only": lambda: KalmanDelayMPC(preview="ar"),
             "MPC, no preview (hold)": lambda: KalmanDelayMPC(preview="hold"),
             "MPC, true future reference (diagnostic)": lambda: KalmanDelayMPC(preview="oracle", oracle=oracle),
             **mpc_baselines()}
    results = {}
    for name, make in ctrls.items():
        results[name] = evaluate(make(), task)
        print(row(name, results[name]))
    after = results["MPC, predicted reference"]
    k0 = int(2.0 * task.control_hz)
    for name in ("MPC, predicted reference", "SMC lam=5 K=0.35 phi=0.02, deviation", "PD kp=500 kd=20"):
        r = results[name]
        e = np.linalg.norm(r.tips[k0:, :2] - r.references[k0:, :2], axis=1) * 1000
        print(f"  {name:<40} RMSE after the first 2 s: {np.sqrt(np.mean(e ** 2)):.3f} mm")

    print("Other tasks and seeds (not used for design or tuning):")
    cases = [("radius 4 mm", dict(radius_m=0.004), 0), ("radius 10 mm", dict(radius_m=0.010), 0),
             ("0.05 Hz", dict(frequency_hz=0.05), 0), ("0.2 Hz", dict(frequency_hz=0.2), 0),
             ("sim seed 1", {}, 1), ("sim seed 2", {}, 2)]
    names = ["MPC, predicted reference", *mpc_baselines()]
    print(f"  {'case':<14}" + "".join(f"{n[:26]:>28}" for n in names))
    makers = {"MPC, predicted reference": lambda: KalmanDelayMPC(), **mpc_baselines()}
    for label, kw, seed in cases:
        t = CircularTrackingTask(**kw)
        print(f"  {label:<14}" + "".join(f"{evaluate(makers[n](), t, seed).rmse_mm:>25.3f} mm" for n in names))
    print(f"  {'tip noise 0.1':<14}" + "".join(
        f"{evaluate(NoisyTip(makers[n]()), task).rmse_mm:>25.3f} mm" for n in names))

    fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
    styles = {"MPC, predicted reference": "C0", "SMC lam=5 K=0.35 phi=0.02, deviation": "C1",
              "PD kp=500 kd=20": "C2"}
    ref = after.references
    ax[0].plot(1000 * (ref[:, 0] - after.home[0]), 1000 * (ref[:, 1] - after.home[1]), "k--", lw=1, label="reference")
    for name, col in styles.items():
        r = results[name]
        ax[0].plot(1000 * (r.tips[:, 0] - r.home[0]), 1000 * (r.tips[:, 1] - r.home[1]), col, lw=1,
                   label=f"{name} ({r.rmse_mm:.2f} mm)")
        e = np.linalg.norm(r.tips[:, :2] - r.references[:, :2], axis=1) * 1000
        ax[1].plot(r.times, e, col, lw=1)
        ax[2].plot(r.times, r.commands[:, 1], col, lw=0.8)
    ax[0].set(xlabel="x [mm]", ylabel="y [mm]", title="Tip path (relative to the settled tip)", aspect="equal")
    ax[0].legend(fontsize=7, loc="upper right")
    ax[1].set(xlabel="time [s]", ylabel="lateral error [mm]", title="Tracking error", yscale="log", ylim=(1e-2, 10))
    ax[2].set(xlabel="time [s]", ylabel="S3 command [psi]", title="S3 command (x channel)")
    for a in ax:
        a.grid(True, alpha=0.3, which="both")
    fig.suptitle("make_calibrated_sim, 7 mm circle at 0.1 Hz (package harness, seed 0)")
    fig.tight_layout()
    out = FIG / "mpc_tracking_calibrated.png"
    fig.savefig(out, dpi=130)
    print(f"  figure: {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
