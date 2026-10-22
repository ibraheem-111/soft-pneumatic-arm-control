"""Do the arm's stiffness and damping change with the operating point? Data-only test of M2.

Fits M2's mechanics row, y_i(t) = K * lag1(w(t), a) (i.e. dy_i/dt = -a y_i + a K w), by
output error to a small step taken at different operating points of make_calibrated_sim
(seed 21, treated as the real robot; 0.1 mm tip noise; only tip position and pressure
sensors are used). K (mm/psi) is the inverse stiffness, a (1/s) stiffness over damping.

Axes and inputs w (measured pressure deviation from the operating point):
    x: S3 step +dp,            w = p_S3,              output y1
    y: S2 +dp, S4 -dp,         w = p_S2 - p_S4,       output y2
    z: S2, S3, S4 all +dp,     w = mean(p_S2..S4),    output y3
Operating points:
    mean inflation b = 2.0 ... 7.5 psi on S2-S4 (straight arm)
    bend offset o = -2 ... +2 psi at b = 4.5 (S2 = 4.5 + o, S4 = 4.5 - o: arm already bent in y)

Usage:
    uv run python scripts/stiffness_vs_pressure.py
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter

from soft_robotic_arm import make_calibrated_sim

DT = 0.01
DP = 0.5
rng = np.random.default_rng(1234)
AXES = {
    "x": (np.array([0.0, DP, 0.0]), lambda p: p[2], 0),
    "y": (np.array([DP, 0.0, -DP]), lambda p: p[1] - p[3], 1),
    "z": (np.array([DP, DP, DP]), lambda p: (p[1] + p[2] + p[3]) / 3, 2),
}


def lag1(w: np.ndarray, a: float) -> np.ndarray:
    phi = np.exp(-a * DT)
    return lfilter([1 - phi], [1, -phi], w)


def fit(base: np.ndarray, axis: str) -> tuple[float, float, float]:
    step, w_of, out = AXES[axis]
    sim = make_calibrated_sim(control_hz=100, seed=21)
    obs = sim.reset()
    for _ in range(1000):
        obs = sim.step(base)
    y0 = np.array(obs["tip_pos"]); p0 = np.array(obs["segment_pressures"])
    ys, ws = [], []
    for _ in range(800):
        obs = sim.step(base + step)
        ys.append(1000 * (np.array(obs["tip_pos"])[out] - y0[out]) + rng.normal(0, 0.1))
        ws.append(w_of(np.array(obs["segment_pressures"]) - p0))
    sim.close()
    ys, ws = np.array(ys), np.array(ws)
    sol = least_squares(lambda th: th[0] * lag1(ws, np.exp(th[1])) - ys, [1.0, np.log(5.0)])
    return sol.x[0], np.exp(sol.x[1]), float(np.sqrt(np.mean(sol.fun**2)))


def main() -> None:
    print("Mean inflation b (straight arm)")
    for axis in AXES:
        for b in (2.0, 3.0, 4.5, 6.0, 7.5):
            K, a, rms = fit(np.full(3, b), axis)
            print(f"  {axis}: b {b:3.1f} psi   K {K:7.3f} mm/psi   a {a:6.2f} 1/s   fit rmse {rms:.3f} mm")
    print("Bend offset o at b = 4.5 (S2 = 4.5 + o, S4 = 4.5 - o)")
    for axis in AXES:
        for o in (-2.0, -1.0, 0.0, 1.0, 2.0):
            K, a, rms = fit(np.array([4.5 + o, 4.5, 4.5 - o]), axis)
            print(f"  {axis}: o {o:+4.1f} psi   K {K:7.3f} mm/psi   a {a:6.2f} 1/s   fit rmse {rms:.3f} mm")


if __name__ == "__main__":
    main()
