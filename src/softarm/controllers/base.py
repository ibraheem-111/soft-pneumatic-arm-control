"""Controller base class: the package harness's interface.

    compute(t, obs, ref) -> (3,) absolute S2-S4 commands in psi

t is the plant clock, obs the simulator observation (tip_pos, tip_vel, segment_pressures,
pouch_pressures, time), ref the tip reference in metres (absolute). Controllers use only
hardware-available signals and never step the simulator. Most take the settled pose from
their first call.
"""

from abc import ABC, abstractmethod

import numpy as np


class Controller(ABC):
    @abstractmethod
    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        """Absolute S2-S4 commands (psi) for this control step."""


def require(model, cls, controller: str) -> None:
    """Raise a clear error when a controller is given a model it cannot use."""
    if not isinstance(model, cls):
        raise TypeError(f"{controller} needs a model of type {cls.__name__} (got {type(model).__name__})")
