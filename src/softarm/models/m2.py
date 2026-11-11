"""M2: the identified actuator plus first-order mechanics per axis (scripts/task_space_model.py).

    tau p' = -p + G u(t - T)          p: measured S2-S4 pressure above the settled value (psi)
    x_i'   = -a_i x_i + a_i (H p)_i   x: tip minus the settled tip (mm), axes x y z

State (x, p) in R^6, both measured (tip, pouch sensors). Parameters: task_space_model.json
"M2_mechanics" (a, H) and "actuator" (G; lag and dead time averaged over segments).
"""

import json
from pathlib import Path

import numpy as np

from softarm.constants import HZ, TASK_SPACE_JSON
from softarm.models.base import FirstOrderModel, LinearModel
from softarm.models.task_space import ActuatorModel, AxisModel


class M2(LinearModel, FirstOrderModel):
    name = "M2"
    label = "M2 (task_space_model.json)"

    def __init__(self, a: np.ndarray, H: np.ndarray, actuator: ActuatorModel):
        self.axes_measured = [AxisModel.unit_dc(float(ai)) for ai in a]
        self.B_measured = np.asarray(H, float)
        self.actuator = actuator
        self.delay_steps = int(round(actuator.delay_s * HZ))

    @classmethod
    def load(cls, path: Path = TASK_SPACE_JSON) -> "M2":
        mech = json.loads(Path(path).read_text())["M2_mechanics"]
        return cls([mech[a]["a"] for a in "xyz"], [mech[a]["H"] for a in "xyz"], ActuatorModel.load(path))

    @property
    def a(self) -> np.ndarray:
        """Rates per axis (1/s), as the controllers have always evaluated them: params(0)[0] = 1/(1/a)."""
        return np.array([ax.params(0.0)[0] for ax in self.axes_measured])

    @property
    def H(self) -> np.ndarray:
        return self.B_measured

    def continuous(self):
        a, H, act = self.a, self.H, self.actuator
        Ac = np.block([[-np.diag(a), np.diag(a) @ H], [np.zeros((3, 3)), -np.eye(3) / act.tau_s]])
        Bc = np.vstack([np.zeros((3, 3)), np.diag(act.gain) / act.tau_s])
        C = np.hstack([np.eye(3), np.zeros((3, 3))])
        return Ac, Bc, C

    def measured_state(self, y_mm, y_prev_mm, p_psi):
        return np.concatenate([y_mm, p_psi])


def load_m2_dict(path: Path = TASK_SPACE_JSON) -> dict:
    """M2 as plain arrays read straight from the JSON (the brief-task benchmark's form):
    H (measured psi -> mm), rates a, actuator gain g, Hc = H * g, mean tau and dead time."""
    d = json.loads(Path(path).read_text())
    m = d["M2_mechanics"]
    H = np.array([m[ax]["H"] for ax in "xyz"])
    a = np.array([m[ax]["a"] for ax in "xyz"])
    g = np.array(d["actuator"]["gain"])
    return dict(H=H, a=a, g=g, Hc=H * g, tau=float(np.mean(d["actuator"]["tau"])),
                dead_s=float(np.mean(d["actuator"]["dead_s"])))
