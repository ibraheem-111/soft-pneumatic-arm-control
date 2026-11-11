"""Physics linearisation of the open-loop plant make_sim() (no controller): the answer key.

Plant: 20 pouch commands u (psi, flattened S1P1..S4P5) -> tip position and pouch
pressure sensors. State x = [qpos (15), qvel (15), p_actual (20)], n = 50.

    dx/dt = A x + B u,   y = C x        (deviations about an equilibrium)

The mechanics rows of A are central finite differences of MuJoCo's forward
dynamics (qacc from mj_forward, with the simulator's own pressure->force map);
the pressure rows are the simulator's first-order lag, written exactly from
cfg.tau_pneumatic. Uses privileged simulator state, so it is ground truth for
checking identified models (scripts/sysid_frf.py), not an identification method itself.
Validation and controllability analysis: scripts/plant_model.py.
"""

import mujoco
import numpy as np

from softarm.constants import HZ as CONTROL_HZ

OUTPUT_NAMES = ["tip x", "tip y", "tip z"] + [f"S{s + 1}P{k + 1} sensor" for s in range(4) for k in range(5)]


def equilibrium(sim, bias: float, settle_s: float = 60.0) -> np.ndarray:
    """Hold every pouch at `bias` psi until the arm is still; return qpos."""
    sim.reset()
    sim.p_actual[:] = bias
    for _ in range(int(settle_s * CONTROL_HZ)):
        sim.step(np.full(sim.p_actual.shape, bias))
    return sim.data.qpos.copy()


def linearize(sim, q0: np.ndarray, bias: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Continuous-time A (50x50), B (50x20), C (23x50) about (q0, qdot=0, p=bias)."""
    model, data = sim.model, sim.data
    nq, nv, npouch = model.nq, model.nv, sim.p_actual.size
    tau = sim.cfg.tau_pneumatic
    p0 = np.full(npouch, bias)

    def set_state(q, v, p):
        data.qpos[:], data.qvel[:] = q, v
        sim.p_actual[:] = p.reshape(sim.p_actual.shape)

    def qacc(q, v, p):
        set_state(q, v, p)
        sim._apply_pressure_wrench()
        mujoco.mj_forward(model, data)
        return data.qacc.copy()

    def y(q, p):
        set_state(q, np.zeros(nv), p)
        mujoco.mj_forward(model, data)
        sensors = sim.p_actual + sim._deformation_pressure_offset()
        return np.concatenate([data.sensordata[0:3], sensors.ravel()])

    def jac(f, x0, eps):
        return np.column_stack([(f(x0 + eps * e) - f(x0 - eps * e)) / (2 * eps) for e in np.eye(x0.size)])

    v0 = np.zeros(nv)
    residual = np.abs(qacc(q0, v0, p0)).max()
    if residual > 1e-8:
        raise RuntimeError(f"not at equilibrium: |qacc| = {residual:.1e}")

    n = nq + nv + npouch
    A = np.zeros((n, n))
    A[:nq, nq:nq + nv] = np.eye(nq)
    A[nq:nq + nv, :nq] = jac(lambda q: qacc(q, v0, p0), q0, 1e-6)
    A[nq:nq + nv, nq:nq + nv] = jac(lambda v: qacc(q0, v, p0), v0, 1e-6)
    A[nq:nq + nv, nq + nv:] = jac(lambda p: qacc(q0, v0, p), p0, 1e-3)
    A[nq + nv:, nq + nv:] = -np.eye(npouch) / tau
    B = np.zeros((n, npouch))
    B[nq + nv:] = np.eye(npouch) / tau
    C = np.zeros((len(OUTPUT_NAMES), n))
    C[:, :nq] = jac(lambda q: y(q, p0), q0, 1e-6)
    C[:, nq + nv:] = jac(lambda p: y(q0, p), p0, 1e-3)
    return A, B, C
