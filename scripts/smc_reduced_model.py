"""Reduced-model sliding-mode controller vs PD on the package's circular tracking task.

Port of Nikoletta's notebook `notebooks/SMC_Controller_withoutdisturbanceobserver.ipynb`
(sections 4-10). The controllers and the reduced-model derivation follow the
notebook; the only additions are the `--gravity` option and the gain sweeps.

Plant and scoring are the package's own harness, not the `make_sim()` benchmark
in AGENTS.md: `soft_robotic_arm.evaluation.evaluate_controller` with the default
`CircularTrackingTask` (7 mm circle, 0.1 Hz, 5 s settle + 30 s tracked, 4.5 psi
baseline on S2-S4) on `make_calibrated_sim(control_hz=100, seed=0)` (3 inputs,
0.5 s transport delay, fitted regulator gain/bias, sealed S1 reservoir).

Reduced model (one bending mode per axis, pressure as a direct input):
    y'' + c y' + k y = b dp
The notebook builds k from the joint springs only. `--gravity` adds the
gravity stiffness of the hanging chain (finite difference of MuJoCo's
qfrc_bias), which the notebook leaves out. The notebook's control law uses the
absolute tip position in the k*y term; `--deviation` uses the deviation from the
tip position at the first call (the settled pose) instead. The notebook's
segment allocator uses each column's torque axis (-sin phi, cos phi) as if it
were the tip direction, which rotates x and y by 90 degrees (+1 psi of dp_x moves
the tip -2.07 mm in y); `--fix-allocator` uses the tip direction (cos phi, sin phi)
measured in the sim (S2 +y, S3 -x, S4 -y), and flips the sign of b, which the
by-hinge convention makes negative while +dp_x then moves the tip toward +x.

Usage:
    uv run python scripts/smc_reduced_model.py             # notebook model, final gains
    uv run python scripts/smc_reduced_model.py --sweep     # reproduce the notebook's PD and SMC sweeps
    uv run python scripts/smc_reduced_model.py --gravity   # same, with gravity in the reduced model
    uv run python scripts/smc_reduced_model.py --sweep --deviation [--gravity]
    uv run python scripts/smc_reduced_model.py --sweep --fix-allocator [--deviation] [--gravity]
"""

import argparse

import mujoco
import numpy as np
from scipy.linalg import eigh

from soft_robotic_arm import make_calibrated_sim
from soft_robotic_arm.evaluation import CircularTrackingTask, TrackingResult, evaluate_controller
from soft_robotic_arm.model import ArmConfig, build_arm_xml

CONTROL_HZ = 100.0


def reduced_model(include_gravity: bool = False) -> tuple[float, float, float]:
    """Return (k, c, b) of the one-mode tip model, derived as in notebook section 7."""
    cfg = ArmConfig()
    model = mujoco.MjModel.from_xml_string(build_arm_xml(cfg))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    mass = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, mass)
    by = [model.jnt_dofadr[model.joint(f"by{k}").id] for k in range(5)]
    m_sub = mass[np.ix_(by, by)]
    k_sub = np.eye(5) * cfg.base_stiffness
    d_sub = np.eye(5) * cfg.base_damping

    if include_gravity:
        # Gravity stiffness of the hanging chain: d(qfrc_bias)/dq at rest (qvel = 0,
        # so qfrc_bias is gravity only). Symmetrized for the eigenproblem.
        bias0 = data.qfrc_bias.copy()
        k_grav = np.zeros((5, 5))
        for j, dof in enumerate(by):
            data.qpos[:] = 0.0
            data.qpos[dof] = 1e-6
            mujoco.mj_forward(model, data)
            k_grav[:, j] = ((data.qfrc_bias - bias0) / 1e-6)[by]
        k_sub = k_sub + 0.5 * (k_grav + k_grav.T)

    # Lever arms d(tip_x)/d(theta_by_k): the forward-kinematics Jacobian at rest.
    tip_adr = model.sensor_adr[model.sensor("tip_pos").id]
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    base_tip = data.sensordata[tip_adr:tip_adr + 3].copy()
    eps = 0.01
    lever = np.zeros(5)
    for k, dof in enumerate(by):
        data.qpos[:] = 0.0
        data.qpos[dof] = eps
        mujoco.mj_forward(model, data)
        lever[k] = (data.sensordata[tip_adr] - base_tip[0]) / eps

    evals, evecs = eigh(k_sub, m_sub)  # M-normalized: v^T M v = 1
    omegas = np.sqrt(np.clip(evals, 0.0, None))
    force_gain = cfg.pressure_gain * cfg.moment_arm  # N m per psi, per level

    # Mode with the largest static tip gain per psi (notebook's selection rule).
    scores = [abs((lever @ evecs[:, r]) * (evecs[:, r] @ np.ones(5))) / max(omegas[r] ** 2, 1e-9)
              for r in range(5)]
    v = evecs[:, int(np.argmax(scores))]
    omega0 = omegas[int(np.argmax(scores))]
    k = float(omega0**2)
    c = float(v @ d_sub @ v)
    b = float((lever @ v) * force_gain * (v @ np.ones(5)))
    return k, c, b


FIX_ALLOCATOR = False


def segment_allocator() -> np.ndarray:
    """Pseudo-inverse mapping a planar (ux, uy) pressure delta onto S2, S3, S4."""
    phis = ArmConfig().col_azimuths()[1:]  # S1 is the sealed reservoir on this plant
    if FIX_ALLOCATOR:
        col_axes = np.stack([np.cos(phis), np.sin(phis)], axis=1)  # tip direction per segment
    else:
        col_axes = np.stack([-np.sin(phis), np.cos(phis)], axis=1)  # notebook: torque axis
    return np.linalg.pinv(col_axes.T)


class PDController:
    """Notebook section 4: task-space PD, allocated across the 3 driven segments."""

    def __init__(self, kp: float = 500.0, kd: float = 20.0, baseline_psi: float = 4.5,
                 max_psi: float = 9.0):
        self.kp, self.kd = kp, kd
        self.baseline_psi, self.max_psi = baseline_psi, max_psi
        self.alloc = segment_allocator()

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        position_error = np.asarray(ref[:2], dtype=float) - obs["tip_pos"][:2]
        velocity_error = -obs["tip_vel"][:2]  # derivative on measurement
        correction = self.kp * position_error + self.kd * velocity_error
        return np.clip(self.baseline_psi + self.alloc @ correction, 0.0, self.max_psi)


class SlidingModeController:
    """Notebook section 8: SMC on y'' = -k y - c y' + b dp, boundary-layer switching."""

    def __init__(self, k: float, c: float, b: float, lam: float = 5.0, K_switch: float = 0.2,
                 phi: float = 0.02, baseline_psi: float = 4.5, max_psi: float = 9.0,
                 deviation: bool = False):
        self.k, self.c, self.b = k, c, b
        self.deviation = deviation
        self._y0: np.ndarray | None = None
        self.lam, self.K_switch, self.phi = lam, K_switch, phi
        self.baseline_psi, self.max_psi = baseline_psi, max_psi
        self.alloc = segment_allocator()
        self._t_hist: list[float] = []
        self._ref_hist: list[np.ndarray] = []

    def _ref_derivatives(self, t: float, ref_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Backward differences over the last 3 calls; zeros until history fills."""
        self._t_hist.append(t)
        self._ref_hist.append(ref_xy.copy())
        if len(self._t_hist) > 3:
            self._t_hist.pop(0)
            self._ref_hist.pop(0)
        if len(self._t_hist) < 2:
            return np.zeros(2), np.zeros(2)
        dt1 = self._t_hist[-1] - self._t_hist[-2]
        ref_vel = (self._ref_hist[-1] - self._ref_hist[-2]) / dt1
        if len(self._t_hist) < 3:
            return ref_vel, np.zeros(2)
        dt0 = self._t_hist[-2] - self._t_hist[-3]
        ref_vel_prev = (self._ref_hist[-2] - self._ref_hist[-3]) / dt0
        return ref_vel, (ref_vel - ref_vel_prev) / dt1

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        target_xy = np.asarray(ref[:2], dtype=float)
        ref_vel, ref_acc = self._ref_derivatives(t, target_xy)
        y, y_dot = obs["tip_pos"][:2], obs["tip_vel"][:2]
        if self.deviation:
            if self._y0 is None:
                self._y0 = y.copy()
            y, target_xy = y - self._y0, target_xy - self._y0
        e = y - target_xy
        e_dot = y_dot - ref_vel
        s = e_dot + self.lam * e
        sat_s = np.clip(s / self.phi, -1.0, 1.0)
        # Notebook default: y is the absolute tip position, not the deviation from home.
        dp_xy = (self.k * y + self.c * y_dot + ref_acc - self.lam * e_dot
                 - self.K_switch * sat_s) / self.b
        return np.clip(self.baseline_psi + self.alloc @ dp_xy, 0.0, self.max_psi)


def evaluate(controller) -> TrackingResult:
    """Run the package harness on the same plant and seed the notebook used."""
    sim = make_calibrated_sim(control_hz=CONTROL_HZ, seed=0)
    try:
        return evaluate_controller(controller, task=CircularTrackingTask(), sim=sim)
    finally:
        sim.close()


def row(label: str, r: TrackingResult) -> str:
    return (f"{label:<34}{r.rmse_mm:>10.2f}{r.max_error_mm:>10.2f}"
            f"{r.phase_lag_s:>10.2f}{r.effort_psi:>10.2f}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--gravity", action="store_true",
                        help="include gravity stiffness in the reduced model")
    parser.add_argument("--sweep", action="store_true",
                        help="run the notebook's PD and SMC gain sweeps")
    parser.add_argument("--deviation", action="store_true",
                        help="SMC uses tip deviation from the settled pose, not absolute position")
    parser.add_argument("--fix-allocator", action="store_true",
                        help="allocate along each segment's tip direction, not its torque axis")
    args = parser.parse_args()
    global FIX_ALLOCATOR
    FIX_ALLOCATOR = args.fix_allocator

    k, c, b = reduced_model(include_gravity=args.gravity)
    if args.fix_allocator:
        b = -b  # +dp along a tip direction moves the tip that way: b > 0
    omega0 = np.sqrt(k)
    zeta = c / (2 * omega0)
    print(f"Reduced model ({'springs + gravity' if args.gravity else 'springs only, as notebook'}):")
    print(f"  y'' + {c:.3f} y' + {k:.3f} y = {b:.4f} dp   "
          f"({omega0 / (2 * np.pi):.3f} Hz, zeta {zeta:.2f}, static gain {1000 * b / k:.2f} mm/psi)")
    print(f"  poles {np.roots([1.0, c, k]).round(3)}")
    home = evaluate(PDController(kp=0.0, kd=0.0)).home
    print(f"  settled tip (circle centre) on make_calibrated_sim: "
          f"x {1000 * home[0]:.2f} mm, y {1000 * home[1]:.2f} mm, z {1000 * home[2]:.1f} mm")

    print(f"\n{'make_calibrated_sim, CircularTrackingTask':<34}{'RMSE mm':>10}{'max mm':>10}"
          f"{'lag s':>10}{'effort':>10}")
    pd_gains = [(250, 20), (100, 20), (500, 60), (500, 20)] if args.sweep else [(500, 20)]
    for kp, kd in pd_gains:
        print(row(f"PD kp={kp} kd={kd}", evaluate(PDController(kp=kp, kd=kd))))
    smc_gains = ([(3.0, 1.0, 0.01), (3.0, 0.2, 0.02), (5.0, 0.2, 0.02), (8.0, 0.2, 0.02),
                  (5.0, 0.1, 0.02), (5.0, 0.35, 0.02)] if args.sweep else [(5.0, 0.2, 0.02)])
    for lam, K_switch, phi in smc_gains:
        smc = SlidingModeController(k, c, b, lam=lam, K_switch=K_switch, phi=phi,
                                    deviation=args.deviation)
        print(row(f"SMC lam={lam} K={K_switch} phi={phi}", evaluate(smc)))


if __name__ == "__main__":
    main()
