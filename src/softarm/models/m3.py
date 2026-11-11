"""M3: M2's actuator with second-order (lumped-mass) mechanics per axis (scripts/task_space_model.py).

    tau p' = -p + G u(t - T)
    x_i'' + 2 zeta_i w_i x_i' + w_i^2 x_i = w_i^2 (H p)_i

State (x, x', p) in R^9. x and p are measured; x' is the backward difference of the measured tip
(a signal the hardware also has). Parameters: task_space_model.json "M3_mechanics" (wn, zeta, H)
and "actuator".
"""

import json
from pathlib import Path

import numpy as np

from softarm.constants import DT, HZ, TASK_SPACE_JSON
from softarm.models.base import LinearModel
from softarm.models.task_space import ActuatorModel


class M3(LinearModel):
    name = "M3"
    label = "M3 (task_space_model.json M3_mechanics)"

    def __init__(self, wn: np.ndarray, zeta: np.ndarray, H: np.ndarray, actuator: ActuatorModel):
        self.wn, self.zeta, self.H = np.asarray(wn, float), np.asarray(zeta, float), np.asarray(H, float)
        self.actuator = actuator
        self.delay_steps = int(round(actuator.delay_s * HZ))

    @classmethod
    def load(cls, path: Path = TASK_SPACE_JSON) -> "M3":
        mech = json.loads(Path(path).read_text())["M3_mechanics"]
        return cls([mech[a]["wn"] for a in "xyz"], [mech[a]["zeta"] for a in "xyz"],
                   [mech[a]["H"] for a in "xyz"], ActuatorModel.load(path))

    def continuous(self):
        w, z, H, act = self.wn, self.zeta, self.H, self.actuator
        O, I = np.zeros((3, 3)), np.eye(3)
        Ac = np.block([[O, I, O], [-np.diag(w**2), -np.diag(2 * z * w), np.diag(w**2) @ H],
                       [O, O, -I / act.tau_s]])
        Bc = np.vstack([O, O, np.diag(act.gain) / act.tau_s])
        return Ac, Bc, np.hstack([I, O, O])

    def measured_state(self, y_mm, y_prev_mm, p_psi):
        yd = np.zeros(3) if y_prev_mm is None else (y_mm - y_prev_mm) / DT
        return np.concatenate([y_mm, yd, p_psi])

    def params(self) -> dict:
        act = self.actuator
        return dict(wn=self.wn.tolist(), zeta=self.zeta.tolist(), H=self.H.tolist(), tau_s=act.tau_s,
                    gain=list(act.gain), delay_s=act.delay_s)
