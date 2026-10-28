"""Qiao, Tao & Zhang (2024) SMC and NDOSMC in task space, plus a delay-aware variant.

Controllers for `make_calibrated_sim` (3 inputs, S2-S4), scored with the package's
own harness (`evaluate_controller`, `CircularTrackingTask`, seed 0). They use only
hardware signals (tip position/velocity, pressure sensors) and a model identified
from data by `scripts/task_space_model.py` (parameters in
`docs/figures/task_space_model.json`); nothing is read from the simulator's model.

State X = tip position minus the settled tip (x, y, z); input u = segment command
minus the 4.5 psi baseline. With virtual inputs v = B u each axis is the paper's
first-order model (eq. 2), written here as
    X_i' = -alpha_i(rho_i) X_i + beta_i(rho_i) v_i + d_i,
i.e. the paper's a_i = -alpha_i = -k_i/c_i and b_i = beta_i = 1/c_i.

Controllers:
  smc       paper eq. 5:  sigma = e, sign switching, eta = (dk|q| + dc|q'|)/c + eps
  ndosmc    paper eqs. 6-8, 12, 14: NDOB with gain l1 + 2 l2 |q|, sat boundary layer
  ndosmc-d  ndosmc on a Smith-predicted state with the identified actuator delay and
            lag (lag inverted), reference extrapolated by the delay

Two printed signs in the paper are inconsistent with its own derivation and are used
corrected: B = +diag(1/c) (eq. 2; needed for eq. 13) and z' = g(-a q - b u - d_hat)
(eq. 7; follows from eq. 6 with z = d_hat - p(q)).

Usage:
    uv run python scripts/ndosmc.py                  # evaluate all controllers
    uv run python scripts/ndosmc.py --sweep          # small gain sweep, every run printed
"""

import argparse
import json
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from soft_robotic_arm import make_calibrated_sim
from soft_robotic_arm.evaluation import CircularTrackingTask, TrackingResult, evaluate_controller

ROOT = Path(__file__).resolve().parent.parent
MODEL_JSON = ROOT / "docs" / "figures" / "task_space_model.json"
CONTROL_HZ = 100.0
BASELINE_PSI = 4.5
MAX_PSI = 9.0


# --------------------------------------------------------------------------- model

@dataclass
class AxisModel:
    """Per-axis c(rho) X' + k(rho) X = v, i.e. alpha = k/c, beta = 1/c (paper eq. 2, 17).

    k_poly and c_poly are quadratics in rho, highest power first. The spreads bound
    |delta k| and |delta c| over the identified set points (used by eq. 5's gain).
    """
    k_poly: tuple[float, float, float]
    c_poly: tuple[float, float, float]
    rho_range: tuple[float, float] = (-np.inf, np.inf)
    k_spread: float = 0.0
    c_spread: float = 0.0

    @classmethod
    def unit_dc(cls, a: float) -> "AxisModel":
        """X' = -a X + a v (the identification script's lag1): k = 1, c = 1/a."""
        return cls(k_poly=(0.0, 0.0, 1.0), c_poly=(0.0, 0.0, 1.0 / a))

    def kc(self, rho: float) -> tuple[float, float]:
        r = float(np.clip(rho, *self.rho_range))
        return float(np.polyval(self.k_poly, r)), float(np.polyval(self.c_poly, r))

    def params(self, rho: float) -> tuple[float, float]:
        k, c = self.kc(rho)
        return k / c, 1.0 / c


@dataclass
class ActuatorModel:
    """Command deviation -> measured pressure deviation per segment: gain * lag(u(t - T))."""
    gain: np.ndarray
    delay_s: float
    tau_s: float


@dataclass
class TaskSpaceModel:
    """Identified by scripts/task_space_model.py; X in mm, pressures in psi."""
    B: np.ndarray                    # (3, 3): v = B u (M1: the static-gain matrix G)
    axes: list[AxisModel]            # M1, x y z
    actuator: ActuatorModel | None = None
    B_measured: np.ndarray | None = None   # M2: v = H (p_meas - p_home)
    axes_measured: list[AxisModel] = field(default_factory=list)
    lpv: bool = True

    @classmethod
    def load(cls, path: Path = MODEL_JSON, lpv: bool = True) -> "TaskSpaceModel":
        d = json.loads(Path(path).read_text())
        m1 = d["M1"]
        axes = []
        for name in ("x", "y", "z"):
            L = m1["local"][name]
            ks = [pt["k"] for pt in L["points"]]
            cs = [pt["c"] for pt in L["points"]]
            if lpv:
                axes.append(AxisModel(tuple(L["k_poly"]), tuple(L["c_poly"]), tuple(L["rho_range"]),
                                      max(ks) - min(ks), max(cs) - min(cs)))
            else:
                axes.append(AxisModel((0.0, 0.0, L["k_mean"]), (0.0, 0.0, L["c_mean"]),
                                      k_spread=max(ks) - min(ks), c_spread=max(cs) - min(cs)))
        act = d["actuator"]
        mech = d["M2_mechanics"]
        return cls(
            B=np.array(m1["G"]),
            axes=axes,
            actuator=ActuatorModel(np.array(act["gain"]), float(np.mean(act["dead_s"])),
                                   float(np.mean(act["tau"]))),
            B_measured=np.array([mech[a]["H"] for a in ("x", "y", "z")]),
            axes_measured=[AxisModel.unit_dc(mech[a]["a"]) for a in ("x", "y", "z")],
            lpv=lpv,
        )

    @staticmethod
    def rho(segment_pressures: np.ndarray) -> np.ndarray:
        """rho_x = p_S1 - p_S3, rho_y = p_S2 - p_S4, rho_z = sum (measured, all four sensors)."""
        p = np.asarray(segment_pressures, float)
        return np.array([p[0] - p[2], p[1] - p[3], p.sum()])


# ---------------------------------------------------------------- shared machinery

class _Base:
    """Common bookkeeping: home pose, reference derivatives, input mapping, clipping."""

    def __init__(self, model: TaskSpaceModel, lam: float):
        self.m = model
        self.lam = lam
        self.B_inv = np.linalg.inv(model.B)
        self.home: np.ndarray | None = None
        self.t0: float | None = None
        self._ref_hist: deque = deque(maxlen=3)
        self._t_hist: deque = deque(maxlen=3)
        self.dt = 1.0 / CONTROL_HZ

    def _frame(self, t: float, obs: dict, ref: np.ndarray):
        if self.home is None:
            self.home = np.asarray(obs["tip_pos"], float).copy()
            self.t0 = t
        q = 1000.0 * (np.asarray(obs["tip_pos"], float) - self.home)      # mm
        qd_dot_meas = 1000.0 * np.asarray(obs["tip_vel"], float)          # mm/s
        r = 1000.0 * (np.asarray(ref, float) - self.home)
        self._ref_hist.append(r.copy())
        self._t_hist.append(t)
        r_dot = np.zeros(3)
        r_ddot = np.zeros(3)
        if len(self._t_hist) >= 2:
            r_dot = (self._ref_hist[-1] - self._ref_hist[-2]) / self.dt
        if len(self._t_hist) == 3:
            r_dot_prev = (self._ref_hist[-2] - self._ref_hist[-3]) / self.dt
            r_ddot = (r_dot - r_dot_prev) / self.dt
        return q, qd_dot_meas, r, r_dot, r_ddot

    def _command(self, v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Map virtual input to clipped segment commands; return (command, applied v)."""
        u = self.B_inv @ v
        cmd = np.clip(BASELINE_PSI + u, 0.0, MAX_PSI)
        return cmd, self.m.B @ (cmd - BASELINE_PSI)


# ------------------------------------------------------------------ paper eq. (5)

class PaperSMC(_Base):
    """Baseline SMC, eq. 5: sigma = e, v = (qd' + alpha q - lam sigma - eta sign sigma)/beta."""

    def __init__(self, model: TaskSpaceModel, lam: float = 2.0, eps: float = 1e-3):
        super().__init__(model, lam)
        self.eps = eps

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        q, q_dot, r, r_dot, _ = self._frame(t, obs, ref)
        rho = self.m.rho(obs["segment_pressures"])
        v = np.zeros(3)
        for i, ax in enumerate(self.m.axes):
            a, b = ax.params(rho[i])
            sigma = q[i] - r[i]
            k, c = ax.kc(rho[i])
            eta = (ax.k_spread * abs(q[i]) + ax.c_spread * abs(q_dot[i])) / abs(c) + self.eps
            v[i] = (r_dot[i] + a * q[i] - self.lam * sigma - eta * np.sign(sigma)) / b
        cmd, _ = self._command(v)
        return cmd


# -------------------------------------------------------- paper eqs. (6)-(8), (12)

class NDOB:
    """Nonlinear disturbance observer, eqs. 6-9, per axis, forward Euler at the control rate."""

    def __init__(self, l1: float, l2: float, dt: float):
        self.l1, self.l2, self.dt = l1, l2, dt
        self.z: np.ndarray | None = None

    def p(self, q: np.ndarray) -> np.ndarray:
        return self.l1 * q + self.l2 * q**2 * np.sign(q)

    def gain(self, q: np.ndarray) -> np.ndarray:
        return self.l1 + 2.0 * self.l2 * np.abs(q)

    def estimate(self, q: np.ndarray) -> np.ndarray:
        if self.z is None:
            self.z = -self.p(q)  # d_hat(0) = 0
        return self.z + self.p(q)

    def update(self, q: np.ndarray, alpha: np.ndarray, beta_v: np.ndarray) -> None:
        """z' = g(-a q - b u - d_hat) with a = -alpha, b u = beta v (corrected eq. 7)."""
        d_hat = self.estimate(q)
        self.z = self.z + self.dt * self.gain(q) * (alpha * q - beta_v - d_hat)


class PaperNDOSMC(_Base):
    """NDOSMC, eq. 12 with the adaptive gain of eq. 14 and a sat boundary layer mu."""

    def __init__(self, model: TaskSpaceModel, lam: float = 2.0, l1: float = 5.0,
                 l2: float = 0.0, Phi: float = 1e-3, eps: float = 1e-3, mu: float = 1e-3,
                 observer: bool = True):
        super().__init__(model, lam)
        self.obs = NDOB(l1, l2, self.dt)
        self.use_observer = observer
        self.Phi, self.eps, self.mu = Phi, eps, mu

    def _law(self, q, r, r_dot, rho, d_hat, t_rel, axes):
        g = self.obs.gain(q)
        v = np.zeros(3)
        alpha = np.zeros(3)
        beta = np.zeros(3)
        for i, ax in enumerate(axes):
            a, b = ax.params(rho[i])
            alpha[i], beta[i] = a, b
            sigma = q[i] - r[i]
            # eq. 14; with the observer off, a constant switching gain eps
            eta = self.Phi * np.exp(-g[i] * t_rel) + self.eps / g[i] if self.use_observer else self.eps
            sat = np.clip(sigma / self.mu, -1.0, 1.0)
            v[i] = (r_dot[i] + a * q[i] - self.lam * sigma - eta * sat - d_hat[i]) / b
        return v, alpha, beta

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        q, _, r, r_dot, _ = self._frame(t, obs, ref)
        rho = self.m.rho(obs["segment_pressures"])
        d_hat = self.obs.estimate(q)
        v, alpha, beta = self._law(q, r, r_dot, rho, d_hat, t - self.t0, self.m.axes)
        cmd, v_applied = self._command(v)
        self.obs.update(q, alpha, beta * v_applied)
        return cmd


# ------------------------------------------------ delay-aware: predictor + lag inverse

class DelayAwareNDOSMC(PaperNDOSMC):
    """NDOSMC on a Smith-predicted state, with the identified actuator delay and lag.

    Mechanics are the model identified from MEASURED pressure (B_measured, axes_measured);
    the actuator model maps segment command -> measured pressure. The outer law asks for an
    effective pressure; the command inverts the lag (u = (p_des + tau p_des')/gain).
    The predictor runs the delay-free model internally and adds (model now - model delayed)
    to the measurement, so the outer loop sees an estimate of X(t + T). The reference is
    extrapolated T ahead from its own backward differences, unless exact_preview=True: then
    the caller passes r(t + T) itself as `ref` (possible only when the trajectory is known in
    advance) and no extrapolation is done.
    """

    def __init__(self, model: TaskSpaceModel, exact_preview: bool = False, **kw):
        super().__init__(model, **kw)
        self.exact_preview = exact_preview
        act = model.actuator
        if act is None or model.B_measured is None or not model.axes_measured:
            raise ValueError("delay-aware controller needs the actuator and measured-pressure models")
        self.act = act
        self.Bp = model.B_measured
        self.Bp_inv = np.linalg.inv(self.Bp)
        self.n_delay = max(1, int(round(act.delay_s * CONTROL_HZ)))
        self.p_model = np.zeros(3)          # lag state of the delay-free model, psi above p_ref
        self.x_model = np.zeros(3)          # delay-free mechanics model
        self.x_hist: deque = deque([np.zeros(3)] * self.n_delay, maxlen=self.n_delay)
        self.p_des_prev: np.ndarray | None = None

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        q, _, r, r_dot, r_ddot = self._frame(t, obs, ref)
        T = self.n_delay * self.dt
        q_pred = q + self.x_model - self.x_hist[0]
        if self.exact_preview:  # ref is already r(t + T)
            r_pred, r_dot_pred = r, r_dot
        else:
            r_pred = r + T * r_dot + 0.5 * T**2 * r_ddot
            r_dot_pred = r_dot + T * r_ddot
        rho = np.zeros(3)  # M2 mechanics are not scheduled
        d_hat = self.obs.estimate(q_pred) if self.use_observer else np.zeros(3)
        v, alpha, beta = self._law(q_pred, r_pred, r_dot_pred, rho, d_hat, t - self.t0,
                                   self.m.axes_measured)
        p_des = self.Bp_inv @ v                       # desired effective pressure above p_ref
        p_des_dot = np.zeros(3) if self.p_des_prev is None else (p_des - self.p_des_prev) / self.dt
        self.p_des_prev = p_des
        p_cmd_eff = p_des + self.act.tau_s * p_des_dot  # lag inverse
        u_cmd = BASELINE_PSI + p_cmd_eff / self.act.gain   # deviation form: bias is in the home pose
        cmd = np.clip(u_cmd, 0.0, MAX_PSI)
        # advance the internal delay-free model with the command actually sent
        p_target = self.act.gain * (cmd - BASELINE_PSI)
        self.p_model = self.p_model + self.dt / self.act.tau_s * (p_target - self.p_model)
        v_model = self.Bp @ self.p_model
        for i, ax in enumerate(self.m.axes_measured):
            a, b = ax.params(rho[i])
            self.x_model[i] += self.dt * (-a * self.x_model[i] + b * v_model[i])
        self.x_hist.append(self.x_model.copy())
        self.obs.update(q_pred, alpha, beta * v_model)
        return cmd


# --------------------------------------------------------------------------- runs

def evaluate(controller) -> TrackingResult:
    sim = make_calibrated_sim(control_hz=CONTROL_HZ, seed=0)
    try:
        return evaluate_controller(controller, task=CircularTrackingTask(), sim=sim)
    finally:
        sim.close()


def row(label: str, r: TrackingResult) -> str:
    return (f"{label:<44}{r.rmse_mm:>9.2f}{r.max_error_mm:>9.2f}"
            f"{r.phase_lag_s:>9.2f}{r.effort_psi:>9.2f}")


SWEEP_SMC = [dict(lam=lam, eps=eps) for lam in (1.0, 2.0, 4.0, 8.0) for eps in (0.1, 1.0)]
# l2 = l1/14 makes the nonlinear gain term equal l1 at |X| = 7 mm (the circle radius).
SWEEP_NDO = [dict(lam=lam, l1=l1, l2=l2f * l1, mu=mu, Phi=1.0, eps=1.0)
             for lam in (1.0, 2.0, 4.0) for l1 in (0.1, 0.2, 0.5, 1.0, 2.0, 5.0)
             for l2f in (0.0, 1.0 / 14.0) for mu in (0.1, 1.0)]
SWEEP_DELAY = SWEEP_NDO + [dict(lam=lam, l1=0.5, l2=0.0, mu=mu, Phi=1.0, eps=1.0)
                           for lam in (6.0, 8.0) for mu in (0.1, 1.0)]
DEFAULT_NDO = dict(lam=2.0, l1=1.0, l2=0.0, mu=1.0, Phi=1.0, eps=1.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--model", type=Path, default=MODEL_JSON)
    parser.add_argument("--sweep", action="store_true", help="run and print every gain setting")
    parser.add_argument("--lti", action="store_true", help="M1 with averaged (unscheduled) k, c")
    args = parser.parse_args()
    model = TaskSpaceModel.load(args.model, lpv=not args.lti)

    print(f"{'make_calibrated_sim seed 0, CircularTrackingTask':<44}{'RMSE':>9}{'max':>9}"
          f"{'lag s':>9}{'effort':>9}")
    smc_grid = SWEEP_SMC if args.sweep else [dict(lam=2.0, eps=0.1)]
    ndo_grid = SWEEP_NDO if args.sweep else [DEFAULT_NDO]
    for g in smc_grid:
        print(row(f"SMC eq5 {g}", evaluate(PaperSMC(model, **g))))
    for g in ndo_grid:
        print(row(f"NDOSMC {g}", evaluate(PaperNDOSMC(model, **g))))
    for g in (SWEEP_DELAY if args.sweep else ndo_grid):
        print(row(f"delay-aware {g}", evaluate(DelayAwareNDOSMC(model, **g))))
    # ablation: predictor + lag inverse with the observer switched off
    for lam in ((1.0, 2.0, 4.0, 8.0) if args.sweep else (4.0,)):
        for eps in (0.1, 1.0):
            g = dict(lam=lam, mu=1.0, eps=eps, observer=False)
            print(row(f"delay-aware, no observer {g}", evaluate(DelayAwareNDOSMC(model, **g))))
    print(row("paper SMC eq5, LTI model (lam 1, eps 0.1)",
              evaluate(PaperSMC(TaskSpaceModel.load(args.model, lpv=False), lam=1.0, eps=0.1))))


if __name__ == "__main__":
    main()
