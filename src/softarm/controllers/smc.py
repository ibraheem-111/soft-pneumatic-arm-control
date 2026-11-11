"""Sliding-mode controllers.

NotebookSMC        Nikoletta's SMC on the one-mode model (notebook section 8), boundary layer
PaperSMC           Qiao, Tao & Zhang (2024) eq. 5 on M1 (rho-scheduled), sign switching
PaperNDOSMC        the same paper's eqs. 6-8, 12, 14: NDOB + adaptive switching gain, sat layer
DelayAwareNDOSMC   PaperNDOSMC's law on a Smith-predicted state with the identified actuator delay
                   and lag (lag inverted); works with any FirstOrderModel (M2, or M1's container).
                   The reference is extrapolated T ahead (Taylor), or with exact_preview=True the
                   caller passes r(t + T) itself (softarm.controllers.preview.Ahead).

Two printed signs in the paper are inconsistent with its own derivation and are used corrected:
B = +diag(1/c) (eq. 2; needed for eq. 13) and z' = g(-a q - b u - d_hat) (eq. 7). Internally
X = tip minus the settled tip (mm), u = command minus 4.5 psi; per axis
X_i' = -alpha_i(rho_i) X_i + beta_i(rho_i) v_i + d_i.
"""

from collections import deque

import numpy as np

from softarm.constants import BASE_PSI, HZ, LIMIT_PSI
from softarm.controllers.base import Controller, require
from softarm.controllers.observers import NDOB
from softarm.controllers.pd import segment_allocator
from softarm.models.base import FirstOrderModel
from softarm.models.one_mode import OneMode

CONTROL_HZ, BASELINE_PSI, MAX_PSI = HZ, BASE_PSI, LIMIT_PSI


# ------------------------------------------------------------------ notebook SMC

class SlidingModeController(Controller):
    """Notebook section 8: SMC on y'' = -k y - c y' + b dp, boundary-layer switching.

    With deviation=False (notebook default) y is the absolute tip position in the k*y term;
    deviation=True uses the deviation from the tip at the first call (the settled pose)."""

    def __init__(self, k: float, c: float, b: float, lam: float = 5.0, K_switch: float = 0.2,
                 phi: float = 0.02, baseline_psi: float = 4.5, max_psi: float = 9.0,
                 deviation: bool = False, corrected_allocator: bool = False):
        self.k, self.c, self.b = k, c, b
        self.deviation = deviation
        self._y0: np.ndarray | None = None
        self.lam, self.K_switch, self.phi = lam, K_switch, phi
        self.baseline_psi, self.max_psi = baseline_psi, max_psi
        self.alloc = segment_allocator(corrected_allocator)
        self._t_hist: list[float] = []
        self._ref_hist: list[np.ndarray] = []

    @classmethod
    def from_model(cls, model: OneMode, corrected_allocator: bool = False, **kw) -> "SlidingModeController":
        """Build on a OneMode model; b's sign follows the allocator (flipped when corrected)."""
        require(model, OneMode, "SlidingModeController")
        return cls(model.k, model.c, model.b_signed(corrected_allocator),
                   corrected_allocator=corrected_allocator, **kw)

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


NotebookSMC = SlidingModeController


# ---------------------------------------------------------------- shared machinery

class _Base(Controller):
    """Common bookkeeping: home pose, reference derivatives, input mapping, clipping."""

    def __init__(self, model, lam: float):
        self.m = model
        self.lam = lam
        B = getattr(model, "B", None)                 # M1's G; only the paper laws map through it
        self.B_inv = None if B is None else np.linalg.inv(B)
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

    def __init__(self, model, lam: float = 2.0, eps: float = 1e-3):
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

class PaperNDOSMC(_Base):
    """NDOSMC, eq. 12 with the adaptive gain of eq. 14 and a sat boundary layer mu.

    self.obs is the disturbance observer (NDOB), not an observation."""

    def __init__(self, model, lam: float = 2.0, l1: float = 5.0,
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
    advance) and no extrapolation is done. observer=False gives the plain delay-aware SMC.
    """

    def __init__(self, model: FirstOrderModel, exact_preview: bool = False, **kw):
        require(model, FirstOrderModel, "DelayAwareNDOSMC")
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

    def predicted_tip(self, q: np.ndarray) -> np.ndarray:
        """Smith prediction of the tip one dead time ahead: q + x_model(t) - x_model(t - T)."""
        return q + self.x_model - self.x_hist[0]

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        q, _, r, r_dot, r_ddot = self._frame(t, obs, ref)
        T = self.n_delay * self.dt
        q_pred = self.predicted_tip(q)
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

    @property
    def gains(self) -> dict:
        return dict(lam=self.lam, mu=self.mu, eps=self.eps, observer=self.use_observer, Phi=self.Phi,
                    l1=self.obs.l1, l2=self.obs.l2, exact_preview=self.exact_preview, dead_time_steps=self.n_delay)
