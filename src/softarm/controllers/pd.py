"""Task-space PD from the course notebook (section 4), and the planar segment allocator.

The notebook's allocator uses each column's torque axis (-sin phi, cos phi) as if it were the tip
direction, which rotates x and y by 90 degrees. allocator="corrected" uses the tip direction
(cos phi, sin phi) measured in the sim (S2 +y, S3 -x, S4 -y).
"""

import numpy as np

from soft_robotic_arm.model import ArmConfig
from softarm.controllers.base import Controller


def segment_allocator(corrected: bool = False) -> np.ndarray:
    """Pseudo-inverse mapping a planar (ux, uy) pressure delta onto S2, S3, S4."""
    phis = ArmConfig().col_azimuths()[1:]  # S1 is the sealed reservoir on this plant
    if corrected:
        col_axes = np.stack([np.cos(phis), np.sin(phis)], axis=1)  # tip direction per segment
    else:
        col_axes = np.stack([-np.sin(phis), np.cos(phis)], axis=1)  # notebook: torque axis
    return np.linalg.pinv(col_axes.T)


class PDController(Controller):
    """Notebook section 4: task-space PD, allocated across the 3 driven segments."""

    def __init__(self, kp: float = 500.0, kd: float = 20.0, baseline_psi: float = 4.5,
                 max_psi: float = 9.0, corrected_allocator: bool = False):
        self.kp, self.kd = kp, kd
        self.baseline_psi, self.max_psi = baseline_psi, max_psi
        self.alloc = segment_allocator(corrected_allocator)

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        position_error = np.asarray(ref[:2], dtype=float) - obs["tip_pos"][:2]
        velocity_error = -obs["tip_vel"][:2]  # derivative on measurement
        correction = self.kp * position_error + self.kd * velocity_error
        return np.clip(self.baseline_psi + self.alloc @ correction, 0.0, self.max_psi)
