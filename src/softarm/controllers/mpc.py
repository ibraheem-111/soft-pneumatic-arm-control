"""Delay-compensated, unconstrained MPC on any LinearModel whose state is measured (M2, M3).

Every 10 ms step k (state X from model.measured_state, input u = command - 4.5 psi):
  1. X_d = (A, B): the model discretised with zero-order hold; u acts after N = model.delay_steps.
  2. Predict across the dead time with the commands already sent:
     X(k+N) = A^N X(k) + sum_i A^(N-1-i) B u(k-N+i).
  3. Model-error offset w: tip measured now minus the tip predicted for now N steps ago,
     low-pass filtered (time constant offset_tau_s), held constant over the horizon.
  4. Minimise sum_{i=0..H-1} |C X(k+N+1+i) + w - r(k+N+i)|^2 + R_u sum |u(k+i) - u(k+i-1)|^2
     over u(k..k+H-1) (unconstrained, closed form, first move applied, then clipped to
     0-9 psi; the clipped command enters step 2). r: the known reference, ref_mm[k] in mm
     relative to the settled tip (the harness's reference at sample k); `ref` passed to
     compute() is not used. Output k+N+1+i is paired with r(k+N+i) because the harness scores
     the tip after step j against r(j).
"""

import numpy as np
from scipy.linalg import expm

from softarm.constants import BASE_PSI, DT, LIMIT_PSI
from softarm.controllers.base import Controller, require
from softarm.models.base import LinearModel


class DelayMPC(Controller):
    """MPC with the reference known in advance. ref_mm: (n, 3) mm from home, n >= run + horizon + delay."""

    def __init__(self, model: LinearModel, ref_mm: np.ndarray, horizon: int, r_u: float,
                 offset_tau_s: float = 0.5):
        require(model, LinearModel, "DelayMPC")
        if model.inputs != "segments":
            raise TypeError(f"DelayMPC needs a model driven by the three segment commands (got {model.name})")
        self.model = model
        Ac, Bc, Cm = model.continuous()
        n = Ac.shape[0]
        E = expm(np.block([[Ac, Bc], [np.zeros((3, n + 3))]]) * DT)
        self.A, self.B, self.C = E[:n, :n], E[:n, n:], Cm
        self.n, self.nd, self.h, self.r_u, self.d = n, model.delay_steps, horizon, r_u, ref_mm
        self.alpha = 1.0 - np.exp(-DT / offset_tau_s)
        self.gains = dict(horizon_steps=horizon, horizon_s=horizon * DT, r_u=r_u, dead_time_steps=self.nd,
                          offset_tau_s=offset_tau_s, model=model.label)
        # Prediction matrices over the horizon: y(k+N+1+i) = Phi_i X(k+N) + sum_j Gam_ij u(k+j)
        h = horizon
        Apow = [np.eye(n)]
        for _ in range(max(h, self.nd) + 1):
            Apow.append(self.A @ Apow[-1])
        self.Apow = Apow
        Phi = np.vstack([Cm @ Apow[i + 1] for i in range(h)])
        Gam = np.zeros((3 * h, 3 * h))
        for i in range(h):
            for j in range(i + 1):
                Gam[3 * i:3 * i + 3, 3 * j:3 * j + 3] = Cm @ Apow[i - j] @ self.B
        D = np.eye(3 * h) - np.eye(3 * h, k=-3)
        K = np.linalg.inv(Gam.T @ Gam + r_u * D.T @ D)[:3]                     # first move only
        self.g_r = K @ Gam.T
        self.g_x = -self.g_r @ Phi
        self.g_w = -self.g_r @ np.tile(np.eye(3), (h, 1))
        E1 = np.zeros((3 * h, 3))
        E1[:3] = np.eye(3)
        self.g_u = r_u * K @ D.T @ E1
        self.W = [Apow[self.nd - 1 - i] @ self.B for i in range(self.nd)]       # delay predictor
        self.k = 0
        self.home = self.p0 = self.y_prev = None
        self.buf = [np.zeros(3)] * self.nd
        self.u_prev = np.zeros(3)
        self.w = np.zeros(3)
        self.pred_hist: list = []

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        if self.home is None:
            self.home = np.asarray(obs["tip_pos"], float).copy()
            self.p0 = np.asarray(obs["segment_pressures"], float)[1:4].copy()
        y = 1000.0 * (np.asarray(obs["tip_pos"], float) - self.home)
        X = self.model.measured_state(y, self.y_prev, np.asarray(obs["segment_pressures"], float)[1:4] - self.p0)
        self.y_prev = y
        Xf = self.Apow[self.nd] @ X + sum(Wi @ ui for Wi, ui in zip(self.W, self.buf))
        self.pred_hist.append(self.C @ Xf)                     # predicted tip at k + N
        if len(self.pred_hist) > self.nd:
            self.w += self.alpha * ((y - self.pred_hist.pop(0)) - self.w)
        k, h, d = self.k, self.h, self.d
        r = d[min(k + self.nd, len(d) - h):][:h]                # r(k+N .. k+N+h-1), mm from home
        r = np.vstack([r, np.repeat(r[-1:], h - len(r), axis=0)]) if len(r) < h else r
        u = self.g_r @ r.reshape(-1) + self.g_x @ Xf + self.g_w @ self.w + self.g_u @ self.u_prev
        cmd = np.clip(BASE_PSI + u, 0.0, LIMIT_PSI)
        u_app = cmd - BASE_PSI
        self.buf = self.buf[1:] + [u_app]
        self.u_prev = u_app
        self.k += 1
        return cmd
