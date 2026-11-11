"""Disturbance observers."""

import numpy as np


class NDOB:
    """Nonlinear disturbance observer (Qiao, Tao & Zhang 2024, eqs. 6-9), per axis, forward Euler.

    d_hat = z + p(q), p(q) = l1 q + l2 q|q|, z' = g(q) (alpha q - beta v - d_hat), g = l1 + 2 l2 |q|.
    With l2 = 0 this is d_hat' = l1 (d - d_hat): a first-order filter of time constant 1/l1."""

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
