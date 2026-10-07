"""Linear state-space model of the open-loop plant (no controller), from make_sim().

Plant: 20 pouch commands u (psi, flattened S1P1..S4P5) -> tip position and pouch
pressure sensors. State x = [qpos (15), qvel (15), p_actual (20)], n = 50.

    dx/dt = A x + B u,   y = C x        (deviations about an equilibrium)

The mechanics rows of A are central finite differences of MuJoCo's forward
dynamics (qacc from mj_forward, with the simulator's own pressure->force map);
the pressure rows are the simulator's first-order lag, written exactly from
cfg.tau_pneumatic. Uses privileged simulator state, so it is ground truth for
checking identified models, not an identification method itself.

The script then:
  1. validates the linear model against the nonlinear sim in open loop
     (a pouch step and a random multisine), and saves a figure;
  2. tests controllability / observability with the PBH test per eigenvalue
     (the rank of [B AB ... A^49 B] is numerically meaningless here: poles span
     about -1 to -15000 1/s);
  3. prints Hankel singular values for 20 pouches -> tip x, y.

Usage:
    uv run python scripts/plant_model.py                 # bias 4.5 psi on every pouch
    uv run python scripts/plant_model.py --bias 2.0
    uv run python scripts/plant_model.py --save plant.npz  # A, B, C, x0, u0 (continuous time)
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np
from scipy.linalg import solve_continuous_lyapunov
from scipy.signal import cont2discrete

from soft_robotic_arm import make_sim

CONTROL_HZ = 100.0
FIGURE = Path(__file__).resolve().parent.parent / "docs" / "figures" / "plant_model_validation.png"
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


def compare_open_loop(sim, q0, bias, A, B, C, u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Run the nonlinear sim and the ZOH-discretized linear model on the same input
    sequence u (N, 20) from the equilibrium; return tip x, y of each, in metres."""
    Ad, Bd, *_ = cont2discrete((A, B, C, np.zeros((C.shape[0], B.shape[1]))), 1 / CONTROL_HZ)
    sim.reset()
    sim.data.qpos[:] = q0
    sim.p_actual[:] = bias
    mujoco.mj_forward(sim.model, sim.data)
    tip0 = sim.data.sensordata[0:2].copy()
    x = np.zeros(A.shape[0])
    nonlinear, linear = [], []
    for uk in u:
        nonlinear.append(sim.step(uk.reshape(sim.p_actual.shape))["tip_pos"][:2] - tip0)
        x = Ad @ x + Bd @ (uk - bias)
        linear.append(C[0:2] @ x)
    return np.asarray(nonlinear), np.asarray(linear)


def multisine(n_steps: int, bias: float, amplitude: float, seed: int) -> np.ndarray:
    """Independent random multisine on each pouch, 0.05-2 Hz, peak <= amplitude."""
    rng = np.random.default_rng(seed)
    t = np.arange(n_steps) / CONTROL_HZ
    freqs = np.geomspace(0.05, 2.0, 12)
    u = np.zeros((n_steps, 20))
    for j in range(20):
        phases = rng.uniform(0, 2 * np.pi, freqs.size)
        sig = np.sin(2 * np.pi * freqs[None, :] * t[:, None] + phases).sum(axis=1)
        u[:, j] = sig / np.abs(sig).max()
    return bias + amplitude * u


def pbh_deficient(A: np.ndarray, B: np.ndarray, tol: float = 1e-9) -> list[tuple[complex, int]]:
    """(eigenvalue, number of lost directions) where rank [λI − A, B] < n."""
    n = A.shape[0]
    seen, lost = [], []
    for lam in np.linalg.eigvals(A):
        if any(abs(lam - s) < 1e-6 * max(1.0, abs(lam)) for s in seen):
            continue
        seen.append(lam)
        s = np.linalg.svd(np.hstack([lam * np.eye(n) - A, B]), compute_uv=False)
        k = int(np.sum(s < tol * s[0]))
        if k:
            lost.append((lam, k))
    return lost


def describe(lost: list[tuple[complex, int]]) -> str:
    return ", ".join(f"{lam.real:.4g}{lam.imag:+.3g}j (x{k})" if abs(lam.imag) > 1e-9
                     else f"{lam.real:.4g} (x{k})" for lam, k in lost) or "none"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bias", type=float, default=4.5, help="pressure on every pouch, psi")
    parser.add_argument("--save", help="write A, B, C, x0, u0 (continuous time) to this .npz file")
    args = parser.parse_args()

    sim = make_sim(control_hz=CONTROL_HZ, seed=0)
    q0 = equilibrium(sim, args.bias)
    A, B, C = linearize(sim, q0, args.bias)
    n = A.shape[0]

    lam = np.linalg.eigvals(A)
    print(f"operating point: every pouch at {args.bias} psi, tip at {sim.data.sensordata[0:3]} m")
    print(f"n = {n} states, max Re(eig A) = {lam.real.max():.4f} 1/s (stable: {lam.real.max() < 0})")
    print("distinct poles (1/s):", np.unique(np.round(lam, 2)))

    # 1. Open-loop validation against the nonlinear sim.
    n_steps = int(20 * CONTROL_HZ)
    step = np.full((n_steps, 20), args.bias)
    step[:, 0] += 2.0  # +2 psi on S1P1
    # Full range: each axis's two opposing columns driven in antiphase, 0..9 psi.
    wide = multisine(n_steps, 0.0, args.bias, seed=12)[:, :2]
    full = np.full((n_steps, 20), args.bias)
    for axis, (plus, minus) in enumerate([(0, 2), (1, 3)]):
        full[:, 5 * plus:5 * plus + 5] += wide[:, [axis]]
        full[:, 5 * minus:5 * minus + 5] -= wide[:, [axis]]
    validation = {"step +2 psi on S1P1": step,
                  "multisine ±2 psi, all pouches (seed 11)": multisine(n_steps, args.bias, 2.0, seed=11),
                  f"opposing columns ±{args.bias:g} psi (seed 12)": full}
    fig, axes = plt.subplots(len(validation), 2, figsize=(11, 8), sharex=True)
    t = np.arange(1, n_steps + 1) / CONTROL_HZ
    print("\nopen-loop validation, tip x, y deviation (nonlinear sim vs linear model):")
    for row, (name, u) in enumerate(validation.items()):
        nonlinear, linear = compare_open_loop(sim, q0, args.bias, A, B, C, u)
        err = np.sqrt(np.mean(np.sum((nonlinear - linear) ** 2, axis=1)))
        size = np.sqrt(np.mean(np.sum(nonlinear ** 2, axis=1)))
        print(f"  {name}: error RMS {1e3 * err:.3f} mm vs signal RMS {1e3 * size:.3f} mm"
              f" ({100 * err / size:.1f} %)")
        for col, axis in enumerate("xy"):
            ax = axes[row, col]
            ax.plot(t, 1e3 * nonlinear[:, col], label="nonlinear sim")
            ax.plot(t, 1e3 * linear[:, col], "--", label="linear model")
            ax.set_title(f"{name}: tip {axis}", fontsize=9)
            ax.set_ylabel("deviation [mm]")
    for ax in axes[-1]:
        ax.set_xlabel("time [s]")
    axes[0, 0].legend()
    fig.tight_layout()
    fig.savefig(FIGURE, dpi=120)
    print(f"  figure: {FIGURE.relative_to(FIGURE.parents[2])}")

    # 2. Controllability and observability (PBH test).
    out = {"tip x, y": C[0:2], "tip x, y, z": C[0:3], "tip x, y + 20 pressure sensors": np.vstack([C[0:2], C[3:]])}
    lost_c = pbh_deficient(A, B)
    print(f"\ncontrollable from the 20 pouch commands: {n - sum(k for _, k in lost_c)}/{n};"
          f" uncontrollable modes: {describe(lost_c)}")
    for name, Cm in out.items():
        lost_o = pbh_deficient(A.T, Cm.T)
        print(f"observable from {name}: {n - sum(k for _, k in lost_o)}/{n};"
              f" unobservable modes: {describe(lost_o)}")

    # 3. Hankel singular values, 20 pouches -> tip x, y.
    Wc = solve_continuous_lyapunov(A, -B @ B.T)
    Wo = solve_continuous_lyapunov(A.T, -C[0:2].T @ C[0:2])
    hsv = np.sqrt(np.clip(np.sort(np.linalg.eigvals(Wc @ Wo).real)[::-1], 0, None))
    print("\nHankel singular values, 20 pouches -> tip x, y (normalized to the largest):")
    print(" ", np.array2string(hsv[:16] / hsv[0], precision=2))

    if args.save:
        np.savez(args.save, A=A, B=B, C=C, output_names=OUTPUT_NAMES, x0=q0, u0=np.full(20, args.bias))
        print(f"\nsaved {args.save}")


if __name__ == "__main__":
    main()
