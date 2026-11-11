"""F: per-channel state-space models identified from frequency responses (scripts/sysid_frf.py).

Two independent channels, each driven by one plane input: nu_x = -u3, nu_y = u2 - u4 (psi).
Per channel: x' = A x + B nu(t - d), y = C x, tip in metres; the first state is the measured channel
pressure (p_x = -mean(S3 pouches), p_y = mean(S2 - S4 pouches)). No z. Parameters:
models/sysid_calibrated_v2.json. The state is estimated (Kalman filter in KalmanDelayMPC), not measured.
"""

import json
from pathlib import Path

import numpy as np

from softarm.constants import F_MODEL_JSON
from softarm.models.base import Model, lsim, plane_inputs


class F(Model):
    name = "F"
    label = "F (sysid_calibrated_v2.json)"
    inputs = "channels"
    outputs = ("x", "y")

    def __init__(self, axes: dict, control_hz: float, path: Path = F_MODEL_JSON):
        self.axes, self.control_hz, self.path = axes, control_hz, Path(path)

    @classmethod
    def load(cls, path: Path = F_MODEL_JSON) -> "F":
        d = json.loads(Path(path).read_text())
        return cls(d["axes"], d["control_hz"], path)

    def channel(self, name: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
        """(A, B (n, 1), C (1, n), delay samples) of channel "x" or "y"; output in metres."""
        ax = self.axes[name]
        return (np.array(ax["A"]), np.array(ax["B"])[:, None], np.array(ax["C"])[None, :],
                int(ax["delay_samples"]))

    def simulate(self, u: np.ndarray) -> np.ndarray:
        """Free run from rest under segment deviations u (n, 3); tip (n, 3) mm, z = nan."""
        nu = plane_inputs(u)
        out = np.full((len(u), 3), np.nan)
        for i, name in enumerate(("x", "y")):
            A, B, C, d = self.channel(name)
            out[:, i] = 1000.0 * lsim(A, B, C, nu[:, [i]], delay=d)[:, 0]
        return out
