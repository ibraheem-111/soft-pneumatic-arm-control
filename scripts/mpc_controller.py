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
     The future reference r(n+d+i) is predicted from the reference's own history
     (AR(2) with intercept on samples 0.1 s apart, exact for a sinusoid plus offset);
     nothing about the task's trajectory is assumed.
  4. Channels to segments: S2 = b + uy/2, S3 = b - ux, S4 = b - uy/2 (the same map the
     model was identified with), clipped to 0-9 psi.

Controller interface: the package harness's compute(t, obs, ref) -> (3,) for S2-S4.
Scored with soft_robotic_arm.evaluation.evaluate_controller (CircularTrackingTask on
make_calibrated_sim, seed 0), against the PD and SMC baselines from
scripts/smc_reduced_model.py (corrected allocator).

Usage:
    uv run python scripts/mpc_controller.py            # evaluate and compare (about 2 min)
    uv run python scripts/mpc_controller.py --tune     # sweep rho on a separate tuning task
"""

import argparse
import json
import sys
from collections import deque
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete

from soft_robotic_arm import make_calibrated_sim
from soft_robotic_arm.evaluation import CircularTrackingTask, evaluate_controller, make_reference

sys.path.insert(0, str(Path(__file__).resolve().parent))
import smc_reduced_model as smc  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = ROOT / "models" / "sysid_calibrated_v2.json"
FIG_DIR = ROOT / "docs" / "figures"
COMMAND_LIMIT_PSI = 9.0
HORIZON = 300             # MPC horizon after the delay, samples (3 s)
RHO = 1e-5                # weight on command changes [m^2 / psi^2]; --tune: largest rho within 3 % of the best RMSE
TUNE_TASK = {"radius_m": 0.005, "frequency_hz": 0.07}   # tuning only; never the scored task
TUNE_SEED = 5


def segment_sensor_matrix() -> np.ndarray:
    """(2, 20): measured channel pressure px = -mean(S3), py = mean(S2 - S4) over levels."""
    S = np.zeros((2, 4, 5))
    S[0, 2, :] = -1 / 5
    S[1, 1, :], S[1, 3, :] = 1 / 5, -1 / 5
    return S.reshape(2, 20)


class AxisController:
    """Kalman filter + delay predictor + unconstrained MPC for one axis."""

    def __init__(self, axis: dict, dt: float, rho: float = RHO, horizon: int = HORIZON,
                 sd_input: float = 0.05, sd_offset: float = 1e-5, sd_tip: float = 1e-4, sd_p: float = 2e-3):
        A, B, C = np.array(axis["A"]), np.array(axis["B"])[:, None], np.array(axis["C"])[None, :]
        Ad, Bd, *_ = cont2discrete((A, B, C, np.zeros((1, 1))), dt)
        Bd = Bd[:, 0]
        n, d = A.shape[0], int(axis["delay_samples"])
        self.n, self.d, self.Ad, self.Bd = n, d, Ad, Bd

        # Kalman filter on z = [x; tip offset]; measurements [tip, channel pressure]
        Az = np.block([[Ad, np.zeros((n, 1))], [np.zeros((1, n)), np.ones((1, 1))]])
        self.Az, self.Bz = Az, np.append(Bd, 0.0)
        e1 = np.eye(n)[0]
        H = np.vstack([np.append(C[0], 1.0), np.append(e1, 0.0)])
        Q = np.zeros((n + 1, n + 1))
        Q[:n, :n] = sd_input ** 2 * np.outer(Bd, Bd) + 1e-12 * np.eye(n)
        Q[n, n] = sd_offset ** 2
        R = np.diag([sd_tip ** 2, sd_p ** 2])
        P = solve_discrete_are(Az.T, H.T, Q, R)
        self.H, self.L = H, P @ H.T @ np.linalg.inv(H @ P @ H.T + R)

        # delay predictor: x_{n+d} = Ad^d x_n + W [u_{n-d}, ..., u_{n-1}]
        self.Ad_d = np.linalg.matrix_power(Ad, d)
        self.W = np.column_stack([np.linalg.matrix_power(Ad, d - 1 - i) @ Bd for i in range(d)]) if d else np.zeros((n, 0))

        # MPC on outputs C x_{n+d+1+i}, i = 0..H-1, driven by u_{n+j}, j <= i
        powers = [np.eye(n)]
        for _ in range(horizon):
            powers.append(Ad @ powers[-1])
        Phi = np.array([C[0] @ powers[i + 1] for i in range(horizon)])
        markov = np.array([C[0] @ powers[k] @ Bd for k in range(horizon)])
        Gamma = np.zeros((horizon, horizon))
        for i in range(horizon):
            Gamma[i, :i + 1] = markov[i::-1]
        Dm = np.eye(horizon) - np.eye(horizon, k=-1)
        k1 = np.linalg.solve(Gamma.T @ Gamma + rho * Dm.T @ Dm, np.eye(horizon)[0])   # first row of the inverse
        self.g_r = k1 @ Gamma.T
        self.g_x = -k1 @ Gamma.T @ Phi
        self.g_off = -k1 @ Gamma.T @ np.ones(horizon)
        self.g_u = rho * k1 @ Dm.T @ np.eye(horizon)[0]
        self.horizon = horizon

        self.z = np.zeros(n + 1)
        self.buf = deque([0.0] * d, maxlen=d) if d else deque()
        self.u_prev = 0.0

    def step(self, tip: float, pressure: float, r_future: np.ndarray) -> float:
        self.z = self.z + self.L @ (np.array([tip, pressure]) - self.H @ self.z)
        x, offset = self.z[:self.n], self.z[self.n]
        x_d = self.Ad_d @ x + (self.W @ np.array(self.buf) if self.d else 0.0)
        return float(self.g_r @ r_future + self.g_x @ x_d + self.g_off * offset + self.g_u * self.u_prev)

    def commit(self, u_applied: float) -> None:
        u_delayed = self.buf[0] if self.d else u_applied
        self.z = self.Az @ self.z + self.Bz * u_delayed
        if self.d:
            self.buf.append(u_applied)
        self.u_prev = u_applied


class RefPredictor:
    """Predicts r(n+k) from the reference's own history.

    mode "ar": AR(2) with intercept on samples `stride` apart over the last `window`
    samples (exact for a sinusoid plus offset), interpolated back to the control rate.
    It holds the last value until 5 coarse samples exist, or when the fitted AR poles
    lie outside the unit circle (a growing prediction).
    mode "hold": r(n+k) = r(n), i.e. no preview."""

    def __init__(self, mode: str = "ar", stride: int = 10, window: int = 400):
        self.mode, self.stride, self.window = mode, stride, window
        self.hist: list[float] = []

    def update(self, r: float) -> None:
        self.hist.append(r)
        if len(self.hist) > self.window + 1:
            self.hist.pop(0)

    def predict(self, start: int, length: int) -> np.ndarray:
        r_now = self.hist[-1]
        hold = np.full(length, r_now)
        coarse = np.array(self.hist[::-1][::self.stride][::-1])
        if self.mode == "hold" or coarse.size < 5 or np.ptp(coarse) < 1e-9:
            return hold
        X = np.column_stack([np.ones(coarse.size - 2), coarse[1:-1], coarse[:-2]])
        c, a1, a2 = np.linalg.lstsq(X, coarse[2:], rcond=None)[0]
        if np.abs(np.roots([1.0, -a1, -a2])).max() > 1.0 + 1e-3:
            return hold                                       # growing prediction: do not trust it
        n_coarse = (start + length) // self.stride + 2
        z = [coarse[-2], coarse[-1]]
        for _ in range(n_coarse):
            z.append(c + a1 * z[-1] + a2 * z[-2])
        z = np.array(z[1:])                                   # z[0] = r(n), z[k] = r(n + k stride)
        return np.interp(np.arange(start, start + length), self.stride * np.arange(z.size), z)


class DelayMPCController:
    """compute(t, obs, ref) -> (3,) S2-S4 setpoints, for evaluate_controller."""

    def __init__(self, model_path: Path = MODEL_PATH, baseline_psi: float = 4.5, rho: float = RHO,
                 preview: str = "ar", oracle=None):
        model = json.loads(model_path.read_text())
        dt = 1.0 / model["control_hz"]
        self.axes = [AxisController(model["axes"][ch], dt, rho=rho) for ch in ("x", "y")]
        self.pred = [RefPredictor(preview if preview != "oracle" else "hold") for _ in range(2)]
        self.preview, self.oracle = preview, oracle
        self.b = baseline_psi
        self.S = segment_sensor_matrix()
        self.tip0 = self.p0 = None
        self.n = 0

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        tip = np.asarray(obs["tip_pos"][:2], dtype=float)
        p = self.S @ np.asarray(obs["pouch_pressures"], dtype=float).ravel()
        if self.tip0 is None:                 # first call: the arm has settled at the baseline
            self.tip0, self.p0 = tip.copy(), p.copy()
        y, pd_, r = tip - self.tip0, p - self.p0, np.asarray(ref[:2], dtype=float) - self.tip0
        u = np.zeros(2)
        for j, (ax, pr) in enumerate(zip(self.axes, self.pred)):
            pr.update(r[j])
            if self.preview == "oracle":      # diagnostic only: the true future reference
                future = np.array([self.oracle(self.n + ax.d + i)[j] for i in range(ax.horizon)]) - self.tip0[j]
            else:
                future = pr.predict(ax.d, ax.horizon)
            u[j] = ax.step(y[j], pd_[j], future)
        cmd = np.clip(self.b + np.array([u[1] / 2, -u[0], -u[1] / 2]), 0.0, COMMAND_LIMIT_PSI)
        applied = (self.b - cmd[1], cmd[0] - cmd[2])
        for ax, ua in zip(self.axes, applied):
            ax.commit(ua)
        self.n += 1
        return cmd


class NoisyTip:
    """Wraps a controller so that it sees tip position with added white noise (the
    simulator's tip sensor is noise-free; motion capture is not). Scoring still uses
    the true tip."""

    def __init__(self, inner, sd_m: float = 1e-4, seed: int = 0):
        self.inner, self.sd, self.rng = inner, sd_m, np.random.default_rng(seed)

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        noisy = dict(obs)
        noisy["tip_pos"] = np.asarray(obs["tip_pos"], dtype=float) + np.append(self.rng.normal(0, self.sd, 2), 0.0)
        noisy["tip_vel"] = np.asarray(obs["tip_vel"], dtype=float)
        return self.inner.compute(t, noisy, ref)


def baselines() -> dict:
    """PD and the best SMC from smc_reduced_model.py, corrected allocator."""
    smc.FIX_ALLOCATOR = True
    k, c, b = smc.reduced_model(include_gravity=False)
    return {
        "PD kp=500 kd=20": lambda: smc.PDController(kp=500, kd=20),
        "SMC lam=5 K=0.35 phi=0.02, deviation": lambda: smc.SlidingModeController(
            k, c, -b, lam=5.0, K_switch=0.35, phi=0.02, deviation=True),
    }


def evaluate(make_ctrl, task: CircularTrackingTask | None = None, seed: int = 0):
    task = task or CircularTrackingTask()
    sim = make_calibrated_sim(control_hz=task.control_hz, seed=seed)
    try:
        return evaluate_controller(make_ctrl(), task=task, sim=sim)
    finally:
        sim.close()


def oracle_for(task: CircularTrackingTask, seed: int):
    """True future reference for the oracle-preview diagnostic: the harness centres the
    circle on the settled tip, which we reproduce with an identical settle."""
    sim = make_calibrated_sim(control_hz=task.control_hz, seed=seed)
    obs = sim.reset()
    for _ in range(task.settle_steps):
        obs = sim.step(task.baseline_command(COMMAND_LIMIT_PSI))
    home = np.asarray(obs["tip_pos"], dtype=float).copy()
    sim.close()
    return lambda n: make_reference(home, n / task.control_hz, task)


def row(name: str, r) -> str:
    return f"  {name:<40}{r.rmse_mm:>9.3f}{r.max_error_mm:>9.2f}{r.phase_lag_s:>9.2f}{r.effort_psi:>9.2f}"


def tune() -> None:
    task = CircularTrackingTask(**TUNE_TASK)
    print(f"Tuning rho on a separate task (radius {task.radius_m * 1000:g} mm, {task.frequency_hz} Hz, "
          f"sim seed {TUNE_SEED})")
    for rho in (1e-8, 1e-7, 1e-6, 1e-5, 1e-4):
        r = evaluate(lambda: DelayMPCController(rho=rho), task, TUNE_SEED)
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
    ctrls = {"MPC, predicted reference (AR)": lambda: DelayMPCController(),
             "MPC, no preview (hold)": lambda: DelayMPCController(preview="hold"),
             "MPC, true future reference (diagnostic)": lambda: DelayMPCController(preview="oracle", oracle=oracle),
             **baselines()}
    results = {}
    for name, make in ctrls.items():
        results[name] = evaluate(make, task)
        print(row(name, results[name]))
    after = results["MPC, predicted reference (AR)"]
    k0 = int(2.0 * task.control_hz)
    for name in ("MPC, predicted reference (AR)", "SMC lam=5 K=0.35 phi=0.02, deviation", "PD kp=500 kd=20"):
        r = results[name]
        e = np.linalg.norm(r.tips[k0:, :2] - r.references[k0:, :2], axis=1) * 1000
        print(f"  {name:<40} RMSE after the first 2 s: {np.sqrt(np.mean(e ** 2)):.3f} mm")

    print("Other tasks and seeds (not used for design or tuning):")
    cases = [("radius 4 mm", dict(radius_m=0.004), 0), ("radius 10 mm", dict(radius_m=0.010), 0),
             ("0.05 Hz", dict(frequency_hz=0.05), 0), ("0.2 Hz", dict(frequency_hz=0.2), 0),
             ("sim seed 1", {}, 1), ("sim seed 2", {}, 2)]
    names = ["MPC, predicted reference (AR)", *baselines()]
    print(f"  {'case':<14}" + "".join(f"{n[:26]:>28}" for n in names))
    makers = {"MPC, predicted reference (AR)": lambda: DelayMPCController(), **baselines()}
    for label, kw, seed in cases:
        t = CircularTrackingTask(**kw)
        print(f"  {label:<14}" + "".join(f"{evaluate(makers[n], t, seed).rmse_mm:>25.3f} mm" for n in names))
    print(f"  {'tip noise 0.1':<14}" + "".join(
        f"{evaluate(lambda m=makers[n]: NoisyTip(m()), task).rmse_mm:>25.3f} mm" for n in names))

    fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
    styles = {"MPC, predicted reference (AR)": "C0", "SMC lam=5 K=0.35 phi=0.02, deviation": "C1",
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
    out = FIG_DIR / "mpc_tracking_calibrated.png"
    fig.savefig(out, dpi=130)
    print(f"  figure: {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
