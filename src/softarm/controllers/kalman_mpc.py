"""Delay-compensated MPC on F, with a Kalman filter per channel (the model's state is not measured).

Per channel (x, y), every control step:
  1. Kalman filter: correct the state estimate [x; tip offset] with the measured tip and the
     measured channel pressure (pouch sensors).
  2. Delay predictor (Smith-predictor style): x_{n+d} = A_d^d x_n + sum A_d^{d-1-i} B_d u_{n-d+i}.
  3. MPC without constraints: minimise sum_i (C x_{n+d+1+i} + offset - r(n+d+i))^2
     + rho sum_i (u_{n+i} - u_{n+i-1})^2 over the horizon; apply the first move. The future
     reference r(n+d+i) is the true one (preview="oracle", with `oracle(n)` -> absolute tip
     reference in metres at sample n), or predicted from the reference's own history
     (RefPredictor: "periodic", "ar") or held ("hold").
  4. Channels to segments: S2 = b + uy/2, S3 = b - ux, S4 = b - uy/2, clipped to 0-9 psi.
"""

from collections import deque

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete

from softarm.constants import LIMIT_PSI
from softarm.controllers.base import Controller, require
from softarm.controllers.preview import RefPredictor
from softarm.models.f import F

COMMAND_LIMIT_PSI = LIMIT_PSI
HORIZON = 300             # MPC horizon after the delay, samples (3 s)
RHO = 1e-5                # weight on command changes [m^2 / psi^2]; tuned on a separate task
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


class KalmanDelayMPC(Controller):
    """compute(t, obs, ref) -> (3,) S2-S4 setpoints, for evaluate_controller."""

    def __init__(self, model: F | None = None, baseline_psi: float = 4.5, rho: float = RHO,
                 preview: str = "periodic", oracle=None):
        model = model or F.load()
        require(model, F, "KalmanDelayMPC")
        self.model = model
        dt = 1.0 / model.control_hz
        self.axes = [AxisController(model.axes[ch], dt, rho=rho) for ch in ("x", "y")]
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
            if self.preview == "oracle":      # the true future reference
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


def known_reference(ref_mm: np.ndarray, model: F | None = None) -> KalmanDelayMPC:
    """MPC on F with the true future reference ref_mm (mm from the settled tip, sampled at 100 Hz);
    tip0 (the settled tip) is set on its first call."""
    ctl = KalmanDelayMPC(model, preview="oracle")
    ctl.oracle = lambda n: ctl.tip0 + ref_mm[n, :2] / 1000.0
    return ctl
