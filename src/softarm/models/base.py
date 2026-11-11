"""Base classes for plant models.

Model             every model: a name and a label, which inputs it takes ("segments": the three
                  S2-S4 command deviations u = command - 4.5 psi; "channels": the plane inputs
                  nu_x = -u3, nu_y = u2 - u4) and which tip axes it predicts.
LinearModel       a continuous-time state space (A, B, C) with an input dead time. Its output is the
                  tip in mm relative to the settled pose. If the state can be measured
                  (measured_state), the delay-compensated MPC (softarm.controllers.DelayMPC) runs on it.
FirstOrderModel   per-axis first-order mechanics x' = -a x + b (H p) driven by measured pressure,
                  plus an actuator model; what the delay-aware SMC/NDOSMC needs.
"""

from abc import ABC, abstractmethod

import numpy as np
from scipy.signal import cont2discrete

from softarm.constants import DT


class Model(ABC):
    name: str = ""
    label: str = ""
    inputs: str = "segments"            # "segments" (u2, u3, u4) or "channels" (nu_x, nu_y)
    outputs: tuple = ("x", "y", "z")


def plane_inputs(u: np.ndarray) -> np.ndarray:
    """(n, 2) plane inputs (nu_x, nu_y) from segment deviations u = (u2, u3, u4): (-u3, u2 - u4)."""
    return np.column_stack([-u[:, 1], u[:, 0] - u[:, 2]])


def lsim(A, B, C, u: np.ndarray, delay: int = 0) -> np.ndarray:
    """ZOH-discretised x' = A x + B u(t - delay*DT), y = C x, from rest; y after each step."""
    Ad, Bd, *_ = cont2discrete((A, B, C, np.zeros((C.shape[0], B.shape[1]))), DT)
    ud = np.vstack([np.zeros((delay, u.shape[1])), u])[: len(u)]
    x = np.zeros(A.shape[0])
    y = np.empty((len(u), C.shape[0]))
    for k in range(len(u)):
        x = Ad @ x + Bd @ ud[k]
        y[k] = C @ x
    return y


class LinearModel(Model):
    """x' = Ac x + Bc u(t - T), y = C x (tip, mm). delay_steps = T at 100 Hz."""

    delay_steps: int = 0

    @abstractmethod
    def continuous(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(Ac, Bc, C) in continuous time."""

    def measured_state(self, y_mm: np.ndarray, y_prev_mm: np.ndarray | None, p_psi: np.ndarray) -> np.ndarray:
        """State built from hardware signals: tip (mm from home), the previous tip, and the measured
        S2-S4 pressure (psi from the settled value). Models whose state is not measurable raise."""
        raise NotImplementedError(f"{self.name}: state is not measurable; use an observer-based controller")

    def simulate(self, u: np.ndarray, delay: bool = True) -> np.ndarray:
        """Free run from rest under input deviations u (n, 3) segments; tip (n, 3) mm."""
        A, B, C = self.continuous()
        return lsim(A, B, C, u, delay=self.delay_steps if delay else 0)


class FirstOrderModel(Model):
    """Per-axis x_i' = -a_i x_i + b_i v_i with v = H p (p: measured pressure above the settled value).

    Required attributes (the delay-aware SMC reads them): axes_measured (list of AxisModel, one per
    axis, giving (a, b) = params(rho)), B_measured (H, 3x3), actuator (ActuatorModel)."""

    axes_measured: list
    B_measured: np.ndarray
    actuator: object
