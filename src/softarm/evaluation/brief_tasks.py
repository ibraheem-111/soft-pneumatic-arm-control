"""The project brief's test motions (slides 15-20), scaled to what make_calibrated_sim can reach.

Shape and timing follow the brief (docs/assignment.md "Project brief: test motions");
amplitudes come from M2's reach (load_m2_dict: H times actuator gain, 0.6 s lag). Tasks
(build_tasks), references in mm relative to the settled tip:

  axial          open-loop tip path of S4 = 3.6 + 3.6 sin(2 pi 0.5 t) psi, S2/S3 at 4.5 psi
  circle_ol      open-loop tip path of phase-shifted sines on S2-S4 at 0.4 Hz, 4.5 +- 3 psi
  circle_static  0.4 Hz circle, largest radius whose static command stays within 0-9 psi
  circle_dyn     same circle, largest radius M2 can follow at 0.4 Hz in steady state
  triangle       equilateral, vertices 0/120/240 deg, 7.5 s per lap, largest circumradius whose
                 quasi-static command along the edges stays within 0-9 psi

Also: reach_checks (open-loop holds at the limits), delayed_floor (a perfect tracker that sees
the reference through the dead time and lag). Used by scripts/benchmark_slide_tasks.py,
scripts/sweep_preview_gain.py and scripts/benchmark_unified.py.
"""

import numpy as np

from soft_robotic_arm import make_calibrated_sim
from soft_robotic_arm.evaluation import CircularTrackingTask, make_reference

from softarm import constants

HZ = int(constants.HZ)         # samples per second, as an int (array lengths, labels)
SEED = 0
BASE, LIMIT = constants.BASE_PSI, constants.LIMIT_PSI
SPAN = min(BASE, LIMIT - BASE)                 # +-4.5 psi around the baseline
ID_RANGE_PSI = (1.5, 7.5)                      # broadband identification levels, task_space_model.py
AXIAL_F, AXIAL_MEAN, AXIAL_AMP = 0.5, 3.6, 3.6 # slide 15-16: Seg 4, 0.5 Hz, 0-7.2 psi
CIRCLE_F = 0.4                                 # slide 17-18
CIRCLE_OL_AMP = 3.0                            # psi: 1.5-7.5, the identified range
TRI_PERIOD = 7.5                               # slide 19-20, s per lap
AZIMUTH = np.radians([90.0, 180.0, 270.0])     # S2, S3, S4


# ----------------------------------------------------------------- model-based reach

def command_per_mm(m: dict, X: np.ndarray, V: np.ndarray | None = None, omega: float = 0.0) -> np.ndarray:
    """Command deviation (psi, S2-S4) that holds planar X (N, 2) in M2's steady state.

    m = softarm.models.load_m2_dict(). Minimum-norm over S2-S4 (pinv of H's x-y rows). With V
    (N, 2, edge velocity in mm/s) the quasi-static ramp demand H p = X + V/a, g u = p + tau dp/dt
    is used; with omega (rad/s) X is a complex phasor and the result is the amplitude of the
    sinusoidal command.
    """
    Hp = np.linalg.pinv(m["H"][:2])
    if omega:
        p = (X * (1 + 1j * omega / m["a"][:2])) @ Hp.T
        return np.abs(p * (1 + 1j * omega * m["tau"]) / m["g"])
    p = X @ Hp.T
    if V is not None:
        p = p + (V / m["a"][:2]) @ Hp.T + m["tau"] * (V @ Hp.T)
    return p / m["g"]


def triangle_vertices(R: float) -> np.ndarray:
    th = np.radians([0.0, 120.0, 240.0])
    return R * np.stack([np.cos(th), np.sin(th)], 1)


def triangle_xy(t: np.ndarray, R: float) -> tuple[np.ndarray, np.ndarray]:
    """Position (mm) and edge velocity (mm/s) on the triangle V0 -> V1 -> V2 -> V0."""
    V = triangle_vertices(R)
    edge_s = TRI_PERIOD / 3
    s = np.mod(t, TRI_PERIOD) / edge_s
    i = np.minimum(s.astype(int), 2)
    f = (s - i)[:, None]
    a, b = V[i], V[(i + 1) % 3]
    return a + f * (b - a), (b - a) / edge_s


def reach_bounds(m: dict) -> dict:
    th = np.linspace(0, 2 * np.pi, 3601)
    unit = np.stack([np.cos(th), np.sin(th)], 1)
    r_static = SPAN / np.abs(command_per_mm(m, unit)).max()
    w = 2 * np.pi * CIRCLE_F
    phasor = np.array([[1.0, -1j]])        # x = cos, y = sin
    r_dyn = SPAN / command_per_mm(m, phasor, omega=w).max()
    tri_static = SPAN / np.abs(command_per_mm(m, triangle_vertices(1.0))).max()
    tt = np.linspace(0, TRI_PERIOD, 30001)[:-1]
    X, V = triangle_xy(tt, 1.0)
    tri_edge = SPAN / np.abs(command_per_mm(m, X, V)).max()
    return dict(circle_static_mm=float(r_static), circle_dyn_0p4hz_mm=float(r_dyn),
                triangle_static_mm=float(tri_static), triangle_edge_mm=float(tri_edge),
                Hc_mm_per_psi=m["Hc"].tolist())


def floor01(x: float) -> float:
    return float(np.floor(10 * x) / 10)


# ----------------------------------------------------------------------- open loop

def open_loop(cmd_fn, n: int, settle=BASE, seed: int = SEED) -> tuple[np.ndarray, np.ndarray]:
    """Settle 5 s at `settle` psi, apply cmd_fn(t) for n steps; tip displacement (mm), commands."""
    sim = make_calibrated_sim(control_hz=HZ, seed=seed)
    obs = sim.reset()
    for _ in range(int(round(constants.SETTLE_S * HZ))):
        obs = sim.step(np.broadcast_to(np.asarray(settle, float), (3,)).copy())
    home = obs["tip_pos"].copy()
    disp, cmds = np.empty((n, 3)), np.empty((n, 3))
    for k in range(n):
        c = np.clip(np.asarray(cmd_fn(k / HZ), float), 0.0, LIMIT)
        obs = sim.step(c)
        disp[k], cmds[k] = 1000.0 * (obs["tip_pos"] - home), c
    sim.close()
    return disp, cmds


def axial_cmd(hold: float):
    return lambda t: np.array([hold, hold, AXIAL_MEAN + AXIAL_AMP * np.sin(2 * np.pi * AXIAL_F * t)])


def circle_ol_cmd(t: float) -> np.ndarray:
    return BASE + CIRCLE_OL_AMP * np.cos(2 * np.pi * CIRCLE_F * t - AZIMUTH)


def reach_checks(m: dict) -> dict:
    """Numerical reach: 15 s holds at the command limits; full-amplitude 0.4 Hz sine on S3."""
    n = 15 * HZ
    holds = {"+x (S3 0)": [BASE, 0.0, BASE], "-x (S3 9)": [BASE, LIMIT, BASE],
             "+y (S2 9, S4 0)": [LIMIT, BASE, 0.0], "-y (S2 0, S4 9)": [0.0, BASE, LIMIT]}
    out = {}
    for name, c in holds.items():
        d, _ = open_loop(lambda t, c=c: c, n)
        out[name] = dict(measured_mm=d[-HZ:].mean(0).tolist(),
                         predicted_mm=(m["Hc"] @ (np.array(c) - BASE)).tolist())
    d, _ = open_loop(lambda t: [BASE, BASE - SPAN * np.sin(2 * np.pi * CIRCLE_F * t), BASE], 30 * HZ)
    x = d[15 * HZ:, 0]
    pred = command_per_mm(m, np.array([[1.0, 0.0]]), omega=2 * np.pi * CIRCLE_F)[0, 1]
    out["S3 +-4.5 psi sine at 0.4 Hz, x amplitude"] = dict(
        measured_mm=float((x.max() - x.min()) / 2), predicted_mm=float(SPAN / pred))
    return out


# ------------------------------------------------------------------------ scoring

def delayed_floor(disp: np.ndarray, dead_s: float, tau: float, lag: bool) -> np.ndarray:
    """Tip of a perfect tracker that sees the reference through the dead time (and lag)."""
    nd = int(round(dead_s * HZ))
    u = np.vstack([np.zeros((nd, 3)), disp])[: len(disp)]
    if not lag:
        return u
    alpha = 1 - np.exp(-1 / (HZ * tau))
    y = np.empty_like(u)
    prev = np.zeros(3)
    for k, uk in enumerate(u):
        prev = prev + alpha * (uk - prev)
        y[k] = prev
    return y


# ------------------------------------------------------------------------ tasks

def build_tasks(n: int, m: dict) -> tuple[dict, dict]:
    """The five tasks' references (mm, relative to home; n samples) and the chosen sizes.

    m = softarm.models.load_m2_dict(). Each task: ref_mm (n, 3), lap_s, short, title, and cmd
    (n, 3) for the open-loop tasks. Open-loop references are deterministic, so a longer n only
    appends samples.
    """
    bounds = reach_bounds(m)
    r_static, r_dyn = floor01(bounds["circle_static_mm"]), floor01(bounds["circle_dyn_0p4hz_mm"])
    r_tri = floor01(bounds["triangle_edge_mm"])

    # references (mm, relative to home), with their generating commands where open loop
    ax_disp, ax_cmd = open_loop(axial_cmd(BASE), n)
    co_disp, co_cmd = open_loop(circle_ol_cmd, n)
    tt = np.arange(n) / HZ
    circ = lambda r: np.array([1000.0 * (make_reference(np.zeros(3), t,  # noqa: E731
                               CircularTrackingTask(radius_m=r / 1000.0, frequency_hz=CIRCLE_F)))
                               for t in tt])
    tri = np.zeros((n, 3))
    tri[:, :2] = triangle_xy(tt, r_tri)[0]
    tasks = {
        "axial": dict(ref_mm=ax_disp, cmd=ax_cmd, lap_s=1 / AXIAL_F, short="axial\n(S4 sine, open-loop path)",
                      title=f"Axial: open-loop path of S4 = {AXIAL_MEAN}+{AXIAL_AMP} sin(2π·{AXIAL_F}t) psi, "
                            f"S2/S3 at {BASE} psi"),
        "circle_ol": dict(ref_mm=co_disp, cmd=co_cmd, lap_s=1 / CIRCLE_F, short="circle, open-loop\n(ellipse)",
                          title=f"Circular (open-loop path): S2-S4 = {BASE} ± {CIRCLE_OL_AMP} psi, "
                                f"{CIRCLE_F} Hz, phases 90/180/270°"),
        "circle_static": dict(ref_mm=circ(r_static), lap_s=1 / CIRCLE_F,
                              short=f"circle r={r_static:g} mm\n(static-H limit)",
                              title=f"Circular (geometric): r = {r_static:g} mm (M2 static limit), {CIRCLE_F} Hz"),
        "circle_dyn": dict(ref_mm=circ(r_dyn), lap_s=1 / CIRCLE_F,
                           short=f"circle r={r_dyn:g} mm\n(0.4 Hz M2 limit)",
                           title=f"Circular (geometric): r = {r_dyn:g} mm (M2 limit at {CIRCLE_F} Hz)"),
        "triangle": dict(ref_mm=tri, lap_s=TRI_PERIOD, short=f"triangle R={r_tri:g} mm",
                         title=f"Triangular: circumradius {r_tri:g} mm, vertices 0/120/240°, "
                               f"{TRI_PERIOD} s per lap"),
    }
    return tasks, dict(circle_static_r_mm=r_static, circle_dyn_r_mm=r_dyn, triangle_R_mm=r_tri)
