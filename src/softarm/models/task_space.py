"""Task-space building blocks identified by scripts/task_space_model.py, and M1.

AxisModel       one axis of c(rho) X' + k(rho) X = v, i.e. alpha = k/c, beta = 1/c (paper eq. 2, 17)
ActuatorModel   command deviation -> measured pressure deviation per segment: gain * lag(u(t - T))
TaskSpaceModel  M1 (rho-scheduled first-order mechanics on v = G u) together with the M2 parts;
                the container the paper SMC/NDOSMC read (B, axes, rho). M1 is an alias.

Parameters: docs/figures/task_space_model.json. The actuator's per-segment lag and dead time are
averaged to scalars, as every controller has always used them.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from softarm.constants import TASK_SPACE_JSON
from softarm.models.base import FirstOrderModel

MODEL_JSON = TASK_SPACE_JSON


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

    @classmethod
    def load(cls, path: Path = TASK_SPACE_JSON) -> "ActuatorModel":
        act = json.loads(Path(path).read_text())["actuator"]
        return cls(np.array(act["gain"]), float(np.mean(act["dead_s"])), float(np.mean(act["tau"])))


@dataclass
class TaskSpaceModel(FirstOrderModel):
    """Identified by scripts/task_space_model.py; X in mm, pressures in psi."""
    B: np.ndarray                    # (3, 3): v = B u (M1: the static-gain matrix G)
    axes: list[AxisModel]            # M1, x y z
    actuator: ActuatorModel | None = None
    B_measured: np.ndarray | None = None   # M2: v = H (p_meas - p_home)
    axes_measured: list[AxisModel] = field(default_factory=list)
    lpv: bool = True
    name = "M1"
    label = "M1 (task_space_model.json)"

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


M1 = TaskSpaceModel
