"""Nikoletta's one-mode model: one bending mode per plane axis, pressure as a direct input.

    y'' + c y' + k y = b dp          y: tip (m), dp: plane pressure input (psi)

Derived from the MuJoCo arm (notebooks/SMC_Controller_withoutdisturbanceobserver.ipynb, section 7):
k from the joint springs (plus the hanging chain's gravity stiffness with include_gravity=True),
c from the joint damping, b from the lever arms and pressure gain. No actuator dynamics, no z.
The by-hinge convention makes b negative; with the corrected allocator (+dp moves the tip toward +dp)
the sign is flipped (b_signed below).
"""

import mujoco
import numpy as np
from scipy.linalg import eigh

from soft_robotic_arm.model import ArmConfig, build_arm_xml
from softarm.models.base import LinearModel, lsim, plane_inputs


def reduced_model(include_gravity: bool = False) -> tuple[float, float, float]:
    """Return (k, c, b) of the one-mode tip model, derived as in notebook section 7."""
    cfg = ArmConfig()
    model = mujoco.MjModel.from_xml_string(build_arm_xml(cfg))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    mass = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, mass)
    by = [model.jnt_dofadr[model.joint(f"by{k}").id] for k in range(5)]
    m_sub = mass[np.ix_(by, by)]
    k_sub = np.eye(5) * cfg.base_stiffness
    d_sub = np.eye(5) * cfg.base_damping

    if include_gravity:
        # Gravity stiffness of the hanging chain: d(qfrc_bias)/dq at rest (qvel = 0,
        # so qfrc_bias is gravity only). Symmetrized for the eigenproblem.
        bias0 = data.qfrc_bias.copy()
        k_grav = np.zeros((5, 5))
        for j, dof in enumerate(by):
            data.qpos[:] = 0.0
            data.qpos[dof] = 1e-6
            mujoco.mj_forward(model, data)
            k_grav[:, j] = ((data.qfrc_bias - bias0) / 1e-6)[by]
        k_sub = k_sub + 0.5 * (k_grav + k_grav.T)

    # Lever arms d(tip_x)/d(theta_by_k): the forward-kinematics Jacobian at rest.
    tip_adr = model.sensor_adr[model.sensor("tip_pos").id]
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    base_tip = data.sensordata[tip_adr:tip_adr + 3].copy()
    eps = 0.01
    lever = np.zeros(5)
    for k, dof in enumerate(by):
        data.qpos[:] = 0.0
        data.qpos[dof] = eps
        mujoco.mj_forward(model, data)
        lever[k] = (data.sensordata[tip_adr] - base_tip[0]) / eps

    evals, evecs = eigh(k_sub, m_sub)  # M-normalized: v^T M v = 1
    omegas = np.sqrt(np.clip(evals, 0.0, None))
    force_gain = cfg.pressure_gain * cfg.moment_arm  # N m per psi, per level

    # Mode with the largest static tip gain per psi (notebook's selection rule).
    scores = [abs((lever @ evecs[:, r]) * (evecs[:, r] @ np.ones(5))) / max(omegas[r] ** 2, 1e-9)
              for r in range(5)]
    v = evecs[:, int(np.argmax(scores))]
    omega0 = omegas[int(np.argmax(scores))]
    k = float(omega0**2)
    c = float(v @ d_sub @ v)
    b = float((lever @ v) * force_gain * (v @ np.ones(5)))
    return k, c, b


class OneMode(LinearModel):
    """Per plane axis: state (y, y'), input dp (psi, plane channel), output y (m). No dead time."""
    name = "one-mode"
    label = "one-mode (Nikoletta)"
    inputs = "channels"
    outputs = ("x", "y")

    def __init__(self, include_gravity: bool = False):
        self.include_gravity = include_gravity
        self.k, self.c, self.b = reduced_model(include_gravity)
        if include_gravity:
            self.name, self.label = "one-mode+g", "one-mode + gravity"
        self.delay_steps = 0

    def b_signed(self, corrected_allocator: bool) -> float:
        """b as the notebook SMC uses it: flipped with the corrected allocator."""
        return -self.b if corrected_allocator else self.b

    def continuous(self):
        """One plane axis, with the corrected sign |b|."""
        A = np.array([[0.0, 1.0], [-self.k, -self.c]])
        return A, np.array([[0.0], [abs(self.b)]]), np.array([[1.0, 0.0]])

    def simulate(self, u: np.ndarray, delay: bool = True) -> np.ndarray:
        """Free run under segment deviations u (n, 3); tip (n, 3) mm, z = nan."""
        A, B, C = self.continuous()
        dp = plane_inputs(u)
        out = np.full((len(u), 3), np.nan)
        for i in (0, 1):
            out[:, i] = 1000.0 * lsim(A, B, C, dp[:, [i]])[:, 0]
        return out
