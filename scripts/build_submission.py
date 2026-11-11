"""Write the group's submission notebooks, built on the professor's template (notebooks/soft_robotic_arm_v2.ipynb).

    notebooks/Soft_robotic_arm_MPC_M3sim.ipynb   MPC on M3's structure (second-order mechanics), identified on make_sim
    notebooks/Soft_robotic_arm_MPC_M2sim.ipynb   MPC on M2's structure (first-order mechanics), identified on make_sim
    notebooks/Soft_robotic_arm_MPC_OneMode.ipynb MPC on Nikoletta's one-mode model (physics-derived, not identified)

The notebooks differ only in the model: every model-specific text and code block below is written for M3sim,
and M2SIM_EDITS turns it into the M2sim version by exact replacement (each must match once). The OneMode
notebook replaces whole blocks (ONEMODE_BLOCKS), adds a section comparing variants of her model, and leaves
out the make_calibrated_sim section, which runs our M2 model rather than hers.

The template's assessment interface is kept exactly: a clearly labelled controller cell defines
student_controller(reference, obs) -> finite (4, 5) absolute pouch psi in [0, 9] and reset_controller();
the instructor's evaluate_controller, test call and plot cells are copied verbatim. The plant is the
template's make_sim(control_hz=100, seed=2). The notebook is standalone (Colab): no repository imports.

Sections: install and setup (template), plant identification on make_sim, MPC derivation, the controller
cell, the instructor evaluation (template), tuning on separate trajectories, comparison with the template's
PID on more trajectories, optional video (template), and a section on make_calibrated_sim (the professor
allowed either plant) with an adapted evaluator and its reach limit.

Usage:
    uv run python scripts/build_submission.py
"""

import json
from pathlib import Path

import nbformat as nbf

ROOT = Path(__file__).resolve().parent.parent
TEMPLATE = ROOT / "notebooks" / "soft_robotic_arm_v2.ipynb"   # the professor's template, copied unchanged
OUT = {v: ROOT / "notebooks" / f"Soft_robotic_arm_MPC_{v}.ipynb" for v in ("M3sim", "M2sim", "OneMode")}
md, code = nbf.v4.new_markdown_cell, nbf.v4.new_code_cell


def template_cells() -> list[str]:
    return ["".join(c["source"]) for c in json.loads(TEMPLATE.read_text())["cells"]]


INTRO = r"""# Group 5 — Soft Robotic Arm: Model-Predictive Control with Trajectory Preview

Built on the course template (`soft_robotic_arm_v2.ipynb`). The assessment interface is unchanged:
the **controller cell** (§5) defines `student_controller(reference, obs)` and `reset_controller()`, and §6 runs
the instructor's `evaluate_controller` exactly as given.

**Controller.** A model-predictive controller (MPC) on a model of the arm identified from the simulator:
pneumatic lag $\tau\dot p = -p + Gu$ and second-order tip mechanics per axis. Every 10 ms it reads the supplied
trajectory one second ahead, `reference(t + 0.01 … t + 1.00)`, and computes the pressures that make the predicted
tip follow it, in closed form. Opposing segments act as pairs (S1/S3 for x, S2/S4 for y) around a 4.5 psi bias,
and each segment's command goes to all five pouches.

**Result on the instructor's example test** (§6, `make_sim(seed=2)`, 20 s, transient included): see the printed
RMSE; on our runs **0.09 mm RMSE**, against **1.26 mm** for the template's PID example. §8 compares the two on twelve
trajectories; §10 repeats the comparison on `make_calibrated_sim`.

Run all cells in order (Runtime → Run all), about 4 minutes on Colab. §9 (video) is optional and off by default."""

ID_MD = r"""## 3 — The plant model, identified from the simulator

The controller needs a model of how pressure commands move the tip. Measured on `make_sim`:

- **Actuator.** Each segment's measured pressure follows its command as a first-order lag with no dead time:
  $\tau\dot p_s = -p_s + G_s u_s$, $\tau \approx 0.60$ s, $G_s \approx 1.00$.
- **Mechanics.** +1 psi on one segment moves the tip 1.8 mm toward it (S1 +x, S2 +y, S3 −x, S4 −y) and opposing
  segments cancel, at every bias pressure from 1 to 8 psi. Per planar axis the tip follows its pressure input as a
  damped second-order system:
$$\ddot y_i + 2\zeta_i\omega_i\,\dot y_i + \omega_i^2\,y_i = \omega_i^2\,(Hp)_i, \qquad i \in \{x, y\}$$
  ($y$ in mm, $p$ the four measured segment pressures, $H$ a 2×4 map in mm/psi).

The cell below re-identifies all parameters from a 60 s random-step experiment (seed 31: each segment jumps to a
random level in 0–9 psi, held 0.5–3 s) by least squares, then checks the model **free-running** (open loop, no
measurement correction) on data it was not fitted to: another random-step run (seed 32) and a 0.02 → 1 Hz chirp.
A first-order mechanics model ($\dot y_i = -a_i y_i + a_i (Hp)_i$) is fitted for comparison. The controller cell
(§5) contains these numbers, rounded."""

ID_CODE = r'''DT = 0.01


def random_steps(seed, seconds=60.0):
    """Each segment jumps to an independent random level in 0-9 psi, held 0.5-3 s."""
    g = np.random.default_rng(seed)
    u, k = np.empty((int(seconds / DT), 4)), 0
    while k < len(u):
        n = int(g.uniform(0.5, 3.0) / DT)
        u[k:k + n] = g.uniform(0.0, 9.0, 4)
        k += n
    return u


def chirp(seconds=60.0):
    t = np.arange(int(seconds / DT)) * DT
    phase = 2 * np.pi * np.cumsum(0.02 * (1.0 / 0.02) ** (t / seconds)) * DT
    return 4.5 + 4.0 * np.column_stack([np.sin(phase), np.cos(phase), -np.sin(phase), -np.cos(phase)])


def run_open_loop(u, seed):
    """Tip x-y (mm) and measured segment pressures (psi) after each command step."""
    s = make_sim(control_hz=CONTROL_HZ, seed=seed)
    obs, tips, pres = s.reset(), [], []
    for c in u:
        obs = s.step(c)
        tips.append(1000.0 * obs["tip_pos"][:2])
        pres.append(obs["segment_pressures"].copy())
    s.close()
    return np.array(tips), np.array(pres)


u_id = random_steps(31)
tips_id, pres_id = run_open_loop(u_id, seed=2)

# Actuator per segment: p[k+1] = phi p[k] + (1 - phi) G u[k]
tau_fit, gain_fit = [], []
for s in range(4):
    (c1, c2), *_ = np.linalg.lstsq(np.column_stack([pres_id[:-1, s], u_id[1:, s]]), pres_id[1:, s], rcond=None)
    tau_fit.append(-DT / np.log(c1))
    gain_fit.append(c2 / (1 - c1))

# Second-order mechanics per axis: y[k+2] = a1 y[k+1] + a2 y[k] + b . p[k+1]
second = {}
for i in range(2):
    A = np.column_stack([tips_id[1:-1, i], tips_id[:-2, i], pres_id[1:-1]])
    th, *_ = np.linalg.lstsq(A, tips_id[2:, i], rcond=None)
    poles = np.log(np.roots([1, -th[0], -th[1]]).astype(complex)) / DT
    wn = float(np.sqrt(np.prod(np.abs(poles))))
    second[i] = dict(wn=wn, zeta=float(-poles.real.sum() / (2 * wn)), H=th[2:] / (1 - th[0] - th[1]))

# First-order mechanics per axis (for comparison): y[k+1] = phi y[k] + (1 - phi) H . p[k]
first = {}
for i in range(2):
    th, *_ = np.linalg.lstsq(np.column_stack([tips_id[:-1, i], pres_id[:-1]]), tips_id[1:, i], rcond=None)
    first[i] = dict(a=-np.log(th[0]) / DT, H=th[1:] / (1 - th[0]))

print("actuator: tau =", np.round(tau_fit, 3), "s, G =", np.round(gain_fit, 3))
for i, ax in enumerate("xy"):
    print(f"{ax}: second order  omega = {second[i]['wn']:.3f} rad/s, zeta = {second[i]['zeta']:.3f}, "
          f"H = {np.round(second[i]['H'], 3)} mm/psi")
    print(f"   first order   a = {first[i]['a']:.3f} 1/s, H = {np.round(first[i]['H'], 3)} mm/psi")


def free_run(u, order):
    """Simulate the identified model open loop from rest; tip x-y (mm) after each step."""
    from scipy.linalg import expm
    tau, G = np.mean(tau_fit), np.array(gain_fit)
    if order == 2:
        w = np.array([second[i]["wn"] for i in range(2)])
        z = np.array([second[i]["zeta"] for i in range(2)])
        H = np.array([second[i]["H"] for i in range(2)])
        Ac = np.block([[np.zeros((2, 2)), np.eye(2), np.zeros((2, 4))],
                       [-np.diag(w ** 2), -np.diag(2 * z * w), np.diag(w ** 2) @ H],
                       [np.zeros((4, 4)), -np.eye(4) / tau]])
    else:
        a = np.array([first[i]["a"] for i in range(2)])
        H = np.array([first[i]["H"] for i in range(2)])
        Ac = np.block([[-np.diag(a), np.diag(a) @ H], [np.zeros((4, 2)), -np.eye(4) / tau]])
    n = Ac.shape[0]
    Bc = np.vstack([np.zeros((n - 4, 4)), np.diag(G) / tau])
    E = expm(np.block([[Ac, Bc], [np.zeros((4, n + 4))]]) * DT)
    A, B, x, out = E[:n, :n], E[:n, n:], np.zeros(n), []
    for c in u:
        x = A @ x + B @ c
        out.append(x[:2])
    return np.array(out)


rows = []
for name, u in (("random steps, seed 32", random_steps(32)), ("chirp 0.02 -> 1 Hz", chirp())):
    tips_v, _ = run_open_loop(u, seed=3)
    for order in (2, 1):
        e = free_run(u, order) - tips_v
        rows.append(f"| {name} | {'second' if order == 2 else 'first'} order | "
                    f"{np.sqrt(np.mean(e[:, 0] ** 2)):.3f} | {np.sqrt(np.mean(e[:, 1] ** 2)):.3f} | "
                    f"{np.sqrt(np.mean(tips_v ** 2)):.2f} |")
display(Markdown("**Free-run validation** (RMSE, mm)\n\n| Data | Model | x | y | signal RMS |\n|---|---|---|---|---|\n"
                 + "\n".join(rows)))'''

MPC_MD = r"""## 4 — The controller: MPC with trajectory preview

**State and input.** $X = (y, \dot y, p - 4.5) \in \mathbb R^8$: tip x-y (mm, measured), tip velocity (backward
difference of the measured tip), and the four measured segment pressures relative to the 4.5 psi bias. The input
is a pair of differential corrections $\delta = (\delta_x, \delta_y)$: segment commands
$u = 4.5 + (\delta_x, \delta_y, -\delta_x, -\delta_y)$, so S1/S3 bend the arm along x and S2/S4 along y, with the
full ±4.5 psi of each segment available. The model of §3 is discretised with a zero-order hold at 10 ms:
$X_{k+1} = AX_k + B\delta_k$, tip $y = CX$.

**Offset.** The model is not exact. $e_k = y_k - C(AX_{k-1} + B\delta_{k-1})$ is its one-step prediction error;
its low-pass (0.5 s) is an offset $w$ added to every prediction:
$w_k = w_{k-1} + \alpha(e_k - w_{k-1})$, $\alpha = 1 - e^{-0.01/0.5}$.

**Optimisation.** Choose the next $H = 100$ corrections (1 s) to minimise
$$J = \sum_{i=0}^{H-1}\big\|C\hat X_{k+1+i} + w - r(t_k + (i+1)\,\Delta t)\big\|^2 + R_u\sum_{i=0}^{H-1}\big\|\delta_{k+i}-\delta_{k+i-1}\big\|^2$$
The tip after step $k+i$ is compared with the reference at that step's time, exactly as the evaluator scores it.
With $\hat Y = \Phi X_k + \Gamma U$ ($\Phi_i = CA^{i+1}$, $\Gamma_{ij} = CA^{i-j}B$ for $j \le i$, $D$ the
first-difference matrix), setting $\partial J/\partial U = 0$:
$$U^\star = \big(\Gamma^\top\Gamma + R_u D^\top D\big)^{-1}\Big[\Gamma^\top\big(\mathcal R - \Phi X_k - \mathbf 1\otimes w\big) + R_u D^\top e_1\,\delta_{k-1}\Big]$$
Only the first correction is applied, so the law is a fixed linear map computed once at start-up:
$$\delta_k = g_r\,\mathcal R + g_x X_k + g_w\,w + g_u\,\delta_{k-1}, \qquad u = \operatorname{clip}_{[0,9]}\big(4.5 + (\delta_x, \delta_y, -\delta_x, -\delta_y)\big)$$
$g_r$ acts on the whole one-second reference window, so the arm starts moving before the reference does; this is
what removes the lag of a feedback-only controller. $R_u = 0.01$ mm²/psi² is tuned in §7 on trajectories that are
not used for testing.

**Pouches.** Each segment's command is sent to all five pouches. The model and the controller act on segment
pressures; the identification in §3 found the response linear in them.

**Memory.** The controller keeps its last prediction, offset, previous correction and previous tip;
`reset_controller()` clears them before each trial."""

CONTROLLER = r'''# STUDENT CONTROLLER CELL: MPC with trajectory preview (Group 5).
import numpy as np
from scipy.linalg import expm

# Plant model identified on make_sim (section 3), per planar axis x, y.
MODEL = {
    "tau_s": 0.600,                                    # pneumatic lag, s
    "gain": np.array([1.0, 1.0, 1.0, 1.0]),            # command -> measured pressure, S1-S4
    "wn": np.array([7.081, 7.101]),                    # natural frequency, rad/s
    "zeta": np.array([0.699, 0.702]),                  # damping ratio
    "H": np.array([[1.794, -0.009, -1.792, 0.008],     # mm per psi of S1..S4, x row
                   [0.005, 1.794, 0.003, -1.803]]),    # y row
}
BIAS_PSI, LIMIT_PSI, DT_S = 4.5, 9.0, 0.01
PAIRS = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]], float)   # u = BIAS + PAIRS @ (dx, dy)


class PreviewMPC:
    """Unconstrained MPC on the identified model, reading the supplied trajectory HORIZON steps ahead."""

    def __init__(self, horizon=100, r_u=0.01, offset_tau_s=0.5):
        w, z, H, tau = MODEL["wn"], MODEL["zeta"], MODEL["H"], MODEL["tau_s"]
        O = np.zeros
        Ac = np.block([[O((2, 2)), np.eye(2), O((2, 4))],
                       [-np.diag(w ** 2), -np.diag(2 * z * w), np.diag(w ** 2) @ H],
                       [O((4, 4)), -np.eye(4) / tau]])
        Bc = np.vstack([O((4, 4)), np.diag(MODEL["gain"]) / tau]) @ PAIRS
        n, m, h = 8, 2, horizon
        E = expm(np.block([[Ac, Bc], [O((m, n + m))]]) * DT_S)
        self.A, self.B = E[:n, :n], E[:n, n:]
        self.C = np.hstack([np.eye(2), O((2, 6))])
        powers = [np.eye(n)]
        for _ in range(h):
            powers.append(self.A @ powers[-1])
        Phi = np.vstack([self.C @ powers[i + 1] for i in range(h)])
        Gam = np.zeros((2 * h, m * h))
        for i in range(h):
            for j in range(i + 1):
                Gam[2 * i:2 * i + 2, m * j:m * j + m] = self.C @ powers[i - j] @ self.B
        D = np.eye(m * h) - np.eye(m * h, k=-m)
        K = np.linalg.solve(Gam.T @ Gam + r_u * D.T @ D, np.eye(m * h))[:m]   # first move only
        self.g_r = K @ Gam.T
        self.g_x = -self.g_r @ Phi
        self.g_w = -self.g_r @ np.tile(np.eye(2), (h, 1))
        E1 = np.zeros((m * h, m))
        E1[:m] = np.eye(m)
        self.g_u = r_u * K @ D.T @ E1
        self.steps_ahead = (np.arange(h) + 1) * DT_S
        self.alpha = 1.0 - np.exp(-DT_S / offset_tau_s)
        self.reset()

    def reset(self):
        self.tip_prev, self.x_pred = None, None
        self.offset, self.delta_prev = np.zeros(2), np.zeros(2)

    def __call__(self, reference, obs):
        t = float(obs["time"])
        tip = 1000.0 * np.asarray(obs["tip_pos"], float)[:2]                     # mm
        vel = np.zeros(2) if self.tip_prev is None else (tip - self.tip_prev) / DT_S
        self.tip_prev = tip
        X = np.concatenate([tip, vel, np.asarray(obs["segment_pressures"], float) - BIAS_PSI])
        if self.x_pred is not None:                                              # model offset
            self.offset += self.alpha * ((tip - self.C @ self.x_pred) - self.offset)
        window = 1000.0 * np.array([reference(t + s) for s in self.steps_ahead])  # mm, (H, 2)
        delta = (self.g_r @ window.reshape(-1) + self.g_x @ X
                 + self.g_w @ self.offset + self.g_u @ self.delta_prev)
        segments = np.clip(BIAS_PSI + PAIRS @ delta, 0.0, LIMIT_PSI)
        applied = np.array([segments[0] - segments[2], segments[1] - segments[3]]) / 2.0
        self.x_pred = self.A @ X + self.B @ applied
        self.delta_prev = applied
        return np.repeat(segments[:, None], 5, axis=1)                          # (4, 5) pouch psi


_mpc = PreviewMPC()


def reset_controller():
    """Clear all controller memory before a new trial."""
    _mpc.reset()


def student_controller(reference, obs):
    """Track the supplied x-y trajectory; return absolute (4, 5) pouch pressures in psi."""
    return _mpc(reference, obs)


reset_controller()'''

TUNE_MD = r"""## 7 — Tuning $R_u$ on separate trajectories

$R_u$ trades tracking against smooth commands. It is chosen on three trajectories that are **not** used for
testing: the template's tutorial family with seed 11 (12 mm), a smooth random path (seed 12) and random waypoints
(seed 13). Score: mean RMSE over the three with the instructor's `evaluate_controller`. The smallest mean is at
$R_u = 0.01$, the value in the controller cell."""

TRAJ = r'''def tutorial_sines(seed, span=0.012, f=(0.05, 0.09, 0.14)):
    """The template's RandomTrajectoryPDController reference family (sum of three sines), centred at 0."""
    g = np.random.default_rng(seed)
    f, a = np.array(f), g.normal(size=(2, 3))
    a = span * a / np.abs(a).sum(1, keepdims=True)
    ph = g.uniform(0, 2 * np.pi, (2, 3))
    return lambda t: (a * np.sin(2 * np.pi * f * t + ph)).sum(1)


def smooth_random(seed, span=0.010, lo=0.03, hi=0.3):
    """Per axis six random sines (0.03-0.3 Hz), scaled to +-span."""
    g = np.random.default_rng(seed)
    f, ph, a = g.uniform(lo, hi, (2, 6)), g.uniform(0, 2 * np.pi, (2, 6)), g.uniform(0.3, 1, (2, 6))
    raw = lambda t: (a * np.sin(2 * np.pi * f * t + ph)).sum(1)
    scale = span / np.abs(np.array([raw(t) for t in np.arange(0, 20, 0.05)])).max(0)
    return lambda t: raw(t) * scale


def waypoints(seed, span=0.010, lo=1.0, hi=2.5):
    """Straight lines between random points in +-span, each 1-2.5 s; starts at the origin."""
    g = np.random.default_rng(seed)
    T, P = [0.0], [np.zeros(2)]
    while T[-1] < 25:
        T.append(T[-1] + g.uniform(lo, hi))
        P.append(g.uniform(-span, span, 2))
    P = np.array(P)
    return lambda t: np.array([np.interp(t, T, P[:, 0]), np.interp(t, T, P[:, 1])])


def circle(radius, freq):
    return lambda t: radius * np.array([np.cos(2 * np.pi * freq * t), np.sin(2 * np.pi * freq * t)])


def polygon(n, radius, lap_s):
    th = 2 * np.pi * np.arange(n + 1) / n
    v = radius * np.column_stack([np.cos(th), np.sin(th)])

    def fn(t):
        s = (t / lap_s % 1.0) * n
        k = int(s)
        return v[k] + (s - k) * (v[k + 1] - v[k])
    return fn


def saturation_pct(result):
    c = result["commands"]
    return 100.0 * np.mean((c <= 0.0) | (c >= 9.0))'''

TUNE = r'''TUNING = {"tutorial sines, seed 11": tutorial_sines(11), "smooth random, seed 12": smooth_random(12),
          "waypoints, seed 13": waypoints(13)}
grid = (1e-4, 1e-3, 1e-2, 1e-1, 1.0)
rows = []
for order_name, make in (("second order (submitted)", lambda r_u: PreviewMPC(r_u=r_u)),):
    for r_u in grid:
        scores = []
        for ref in TUNING.values():
            ctl = make(r_u)
            scores.append(evaluate_controller(ctl, ref, reset=ctl.reset)["rmse_mm"])
        rows.append(f"| {order_name} | {r_u:g} | " + " | ".join(f"{s:.3f}" for s in scores) + f" | **{np.mean(scores):.3f}** |")
display(Markdown("| Model | R_u | " + " | ".join(TUNING) + " | mean |\n|---|---|---|---|---|---|\n" + "\n".join(rows)))'''

COMPARE_MD = r"""## 8 — Comparison with the template's PID on more trajectories

The template's PID example (§7 of the template, unchanged) and our MPC, each through the instructor's
`evaluate_controller` (`make_sim(seed=2)`, 20 s, RMSE and maximum error including the initial transient). The
trajectories are absolute positions in metres, like the instructor's; some start away from the tip, which sits at
the origin at reset, so their maximum error is the unavoidable jump at $t = 0$. Saturation: % of pouch commands at
0 or 9 psi. None of these trajectories was used for tuning."""

COMPARE = r'''# The template's PID example, unchanged, renamed so it does not replace our controller.
def reset_pid():
    global integral_psi, filtered_velocity_error, previous_reference, previous_time
    integral_psi = np.zeros(2)
    filtered_velocity_error = np.zeros(2)
    previous_reference = None
    previous_time = None


def pid_controller(reference, obs):
    global integral_psi, filtered_velocity_error, previous_reference, previous_time
    kp, ki, kd = 2000.0, 800.0, 150.0
    filter_time_s = 0.02
    bias_psi, correction_limit_psi = 4.5, 4.5
    t = float(obs["time"])
    desired_xy = np.asarray(reference(t), dtype=float)
    error = desired_xy - obs["tip_pos"][:2]
    dt = 0.0 if previous_time is None else t - previous_time
    if dt > 0.0:
        desired_velocity = (desired_xy - previous_reference) / dt
        velocity_error = desired_velocity - obs["tip_vel"][:2]
        alpha = dt / (filter_time_s + dt)
        filtered_velocity_error += alpha * (velocity_error - filtered_velocity_error)
    pd_psi = kp * error + kd * filtered_velocity_error
    candidate_integral = integral_psi.copy()
    if dt > 0.0:
        candidate_integral = np.clip(integral_psi + ki * error * dt, -correction_limit_psi, correction_limit_psi)
    trial_correction = pd_psi + candidate_integral
    pushing_limit = (((trial_correction > correction_limit_psi) & (error > 0.0))
                     | ((trial_correction < -correction_limit_psi) & (error < 0.0)))
    integral_psi = np.where(pushing_limit, integral_psi, candidate_integral)
    dx, dy = np.clip(pd_psi + integral_psi, -correction_limit_psi, correction_limit_psi)
    previous_reference = desired_xy.copy()
    previous_time = t
    segment_pressures = bias_psi + np.array([dx, dy, -dx, -dy])
    return np.repeat(segment_pressures[:, None], 5, axis=1)


TESTS = {
    "instructor example (10 mm sines)": instructor_reference,
    "tutorial sines, seed 7 (15 mm)": tutorial_sines(7, 0.015),
    "smooth random, seed 101": smooth_random(101), "smooth random, seed 102": smooth_random(102),
    "smooth random, seed 103": smooth_random(103),
    "waypoints, seed 101": waypoints(101), "waypoints, seed 102": waypoints(102), "waypoints, seed 103": waypoints(103),
    "circle 10 mm, 0.15 Hz": circle(0.010, 0.15), "triangle 10 mm, 10 s lap": polygon(3, 0.010, 10.0),
    "square 10 mm, 10 s lap": polygon(4, 0.010, 10.0), "fast circle 8 mm, 0.4 Hz": circle(0.008, 0.4),
}
compare = {}
rows = []
for name, ref in TESTS.items():
    pid = evaluate_controller(pid_controller, ref, reset=reset_pid)
    mpc = evaluate_controller(student_controller, ref, reset=reset_controller)
    compare[name] = (pid, mpc)
    rows.append(f"| {name} | {pid['rmse_mm']:.3f} | {pid['max_error_mm']:.2f} | {saturation_pct(pid):.1f} | "
                f"**{mpc['rmse_mm']:.3f}** | {mpc['max_error_mm']:.2f} | {saturation_pct(mpc):.1f} |")
display(Markdown("| Trajectory | PID RMSE | PID max | PID sat % | MPC RMSE | MPC max | MPC sat % |\n"
                 "|---|---|---|---|---|---|---|\n" + "\n".join(rows)))'''

OVERLAY = r'''def overlay(name):
    pid, mpc = compare[name]
    t = mpc["times"]
    ref = 1000 * mpc["references"]
    fig, axs = plt.subplots(1, 2, figsize=(15, 5.5), constrained_layout=True)
    axs[0].plot(ref[:, 0], ref[:, 1], "--", color="0.2", lw=2.5, label="reference")
    axs[0].plot(1000 * pid["tips"][:, 0], 1000 * pid["tips"][:, 1], color="#E69F00", ls="-.", lw=2, label="PID (template)")
    axs[0].plot(1000 * mpc["tips"][:, 0], 1000 * mpc["tips"][:, 1], color="#0072B2", lw=2, label="MPC (ours)")
    axs[0].set(xlabel="x [mm]", ylabel="y [mm]", title=name)
    axs[0].set_aspect("equal", adjustable="datalim")
    for res, color, ls, lab in ((pid, "#E69F00", "-.", "PID"), (mpc, "#0072B2", "-", "MPC")):
        e = 1000 * np.linalg.norm(res["references"] - res["tips"], axis=1)
        axs[1].plot(t, e, color=color, ls=ls, lw=2, label=f"{lab}: RMSE {res['rmse_mm']:.3f} mm")
    axs[1].set(xlabel="Time [s]", ylabel="Planar error [mm]", title="Tracking error", ylim=(0, None))
    for a in axs:
        a.grid(True, alpha=0.3)
        a.legend()
    plt.show()


for name in ("instructor example (10 mm sines)", "waypoints, seed 102", "smooth random, seed 101", "square 10 mm, 10 s lap"):
    overlay(name)'''

CALIB_MD = r"""## 10 — The same approach on `make_calibrated_sim`

The professor allowed testing on either plant. `make_calibrated_sim` is fitted to the hardware: S1 is a sealed
reservoir (only S2–S4 are commanded, as a `(3,)` vector), and commands reach the pouches after a **0.5 s transport
delay**, then the same 0.6 s lag. The instructor's evaluator needs one change for this plant: the `(4, 5)` command's
rows S2–S4 are averaged into the `(3,)` segment command, and row S1 is ignored. All checks are unchanged.

Our controller for this plant is the same MPC idea on the model we identified for it (M2: first-order mechanics,
three inputs), plus a prediction across the dead time with the commands already sent
($\hat X_{k+N} = A^N X_k + \sum_i A^{N-1-i}Bu_{k-N+i}$, $N = 50$), reading the reference 0.5–1.5 s ahead.

**Reach limit.** With S1 sealed, only S3 drives x, so the tip cannot go beyond about **+4.2 mm in x** (measured
below). The instructor's 10 mm references are unreachable on half of every x swing, whatever the controller."""

CALIB_CODE = r'''from soft_robotic_arm import make_calibrated_sim

# Reach from reset: hold each extreme command for 15 s.
rows = []
for label, cmd in (("S3 = 0 psi (S2 = S4 = 4.5)", [4.5, 0.0, 4.5]), ("S3 = 9 psi", [4.5, 9.0, 4.5]),
                   ("S2 = 9, S4 = 0", [9.0, 4.5, 0.0]), ("S2 = 0, S4 = 9", [0.0, 4.5, 9.0])):
    s = make_calibrated_sim(control_hz=100, seed=2)
    obs = s.reset()
    for _ in range(1500):
        obs = s.step(np.array(cmd))
    rows.append(f"| {label} | {1000 * obs['tip_pos'][0]:+.2f} | {1000 * obs['tip_pos'][1]:+.2f} |")
    s.close()
display(Markdown("**Reach of make_calibrated_sim** (tip after 15 s, mm)\n\n| Command | x | y |\n|---|---|---|\n" + "\n".join(rows)))


def evaluate_controller_calibrated(controller, reference, *, reset=None, duration_s=20.0, seed=2):
    """The instructor's evaluate_controller on make_calibrated_sim: rows S2-S4 of the (4, 5) command are
    averaged into the (3,) segment command this plant accepts; row S1 (sealed reservoir) is ignored."""
    if reset is not None:
        reset()

    def supplied_reference(t):
        target = np.array(reference(t), dtype=float, copy=True)
        if target.shape != (2,) or not np.all(np.isfinite(target)):
            raise ValueError("reference(t) must return a finite [x, y] vector in metres")
        return target

    test_sim = make_calibrated_sim(control_hz=100.0, seed=seed)
    try:
        obs = test_sim.reset()
        times, targets, tips, commands = [], [], [], []
        for _ in range(int(round(duration_s * 100.0))):
            controller_obs = {key: obs[key].copy() if isinstance(obs[key], np.ndarray) else obs[key]
                              for key in ("time", "tip_pos", "tip_vel", "pouch_pressures", "segment_pressures")}
            command = np.array(controller(supplied_reference, controller_obs), dtype=float, copy=True)
            if command.shape != (4, 5) or not np.all(np.isfinite(command)):
                raise ValueError("Return a finite (4, 5) pressure array")
            if np.any(command < 0.0) or np.any(command > 9.0):
                raise ValueError("Every pressure command must be between 0 and 9 psi")
            obs = test_sim.step(command[1:4].mean(axis=1))
            times.append(obs["time"])
            targets.append(supplied_reference(obs["time"]))
            tips.append(obs["tip_pos"][:2].copy())
            commands.append(command)
        targets, tips = np.asarray(targets), np.asarray(tips)
        errors_m = np.linalg.norm(targets - tips, axis=1)
        return {"rmse_mm": float(1000 * np.sqrt(np.mean(errors_m ** 2))), "max_error_mm": float(1000 * np.max(errors_m)),
                "times": np.asarray(times), "references": targets, "tips": tips, "commands": np.asarray(commands)}
    finally:
        test_sim.close()


M2_CALIBRATED = {   # identified on make_calibrated_sim (S2, S3, S4 inputs; x, y, z outputs)
    "gain": np.array([1.1198, 1.0964, 1.1640]), "tau_s": 0.5995, "delay_steps": 50,
    "a": np.array([4.7686, 4.6992, 15.4935]),
    "H": np.array([[-0.0033, -1.8121, -0.0020], [1.8231, -0.0017, -1.8133], [-0.4929, -0.4461, -0.4896]]),
}


class DelayPreviewMPC:
    """MPC on M2 for make_calibrated_sim: predicts across the 0.5 s dead time, reads reference(t) 0.5-1.5 s ahead."""

    def __init__(self, horizon=100, r_u=1e-5, offset_tau_s=0.5):
        a, H, O, tau = M2_CALIBRATED["a"], M2_CALIBRATED["H"], np.zeros((3, 3)), M2_CALIBRATED["tau_s"]
        Ac = np.block([[-np.diag(a), np.diag(a) @ H], [O, -np.eye(3) / tau]])
        Bc = np.vstack([O, np.diag(M2_CALIBRATED["gain"]) / tau])
        E = expm(np.block([[Ac, Bc], [np.zeros((3, 9))]]) * DT_S)
        A, B = E[:6, :6], E[:6, 6:]
        self.C, h, nd = np.hstack([np.eye(3), O]), horizon, M2_CALIBRATED["delay_steps"]
        self.h, self.nd = h, nd
        P = [np.eye(6)]
        for _ in range(max(h, nd) + 1):
            P.append(A @ P[-1])
        Phi = np.vstack([self.C @ P[i + 1] for i in range(h)])
        Gam = np.zeros((3 * h, 3 * h))
        for i in range(h):
            for j in range(i + 1):
                Gam[3 * i:3 * i + 3, 3 * j:3 * j + 3] = self.C @ P[i - j] @ B
        D = np.eye(3 * h) - np.eye(3 * h, k=-3)
        K = np.linalg.solve(Gam.T @ Gam + r_u * D.T @ D, np.eye(3 * h))[:3]
        self.g_r, E1 = K @ Gam.T, np.zeros((3 * h, 3))
        E1[:3] = np.eye(3)
        self.g_x, self.g_w = -self.g_r @ Phi, -self.g_r @ np.tile(np.eye(3), (h, 1))
        self.g_u = r_u * K @ D.T @ E1
        self.AN, self.W = P[nd], np.hstack([P[nd - 1 - i] @ B for i in range(nd)])
        self.alpha = 1 - np.exp(-DT_S / offset_tau_s)
        self.reset()

    def reset(self):
        self.home, self.p0 = None, None
        self.buf, self.u_prev, self.w, self.pred = np.zeros((self.nd, 3)), np.zeros(3), np.zeros(3), []

    def __call__(self, reference, obs):
        t = float(obs["time"])
        tip, seg = np.asarray(obs["tip_pos"], float), np.asarray(obs["segment_pressures"], float)[1:4]
        if self.home is None:
            self.home, self.p0 = tip.copy(), seg.copy()
        y = 1000 * (tip - self.home)
        X = np.concatenate([y, seg - self.p0])
        Xf = self.AN @ X + self.W @ self.buf.reshape(-1)
        self.pred.append(self.C @ Xf)
        if len(self.pred) > self.nd:
            self.w += self.alpha * ((y - self.pred.pop(0)) - self.w)
        r = np.zeros((self.h, 3))
        times = t + (self.nd + 1 + np.arange(self.h)) * DT_S
        r[:, :2] = 1000 * (np.array([reference(s) for s in times]) - self.home[:2])
        r[:, 2] = (self.C @ Xf)[2] + self.w[2]          # z is not scored: aim it at its own prediction
        u = self.g_r @ r.reshape(-1) + self.g_x @ Xf + self.g_w @ self.w + self.g_u @ self.u_prev
        cmd = np.clip(4.5 + u, 0.0, LIMIT_PSI)
        applied = cmd - 4.5
        self.buf, self.u_prev = np.vstack([self.buf[1:], applied]), applied
        out = np.zeros((4, 5))
        out[1:4] = cmd[:, None]                         # S1 row unused on this plant
        return out


calibrated_mpc = DelayPreviewMPC()
rows = []
for name in ("instructor example (10 mm sines)", "smooth random, seed 101", "waypoints, seed 101", "waypoints, seed 102",
             "square 10 mm, 10 s lap"):
    pid = evaluate_controller_calibrated(pid_controller, TESTS[name], reset=reset_pid)
    mpc = evaluate_controller_calibrated(calibrated_mpc, TESTS[name], reset=calibrated_mpc.reset)
    rows.append(f"| {name} | {pid['rmse_mm']:.3f} | {pid['max_error_mm']:.2f} | "
                f"**{mpc['rmse_mm']:.3f}** | {mpc['max_error_mm']:.2f} |")
display(Markdown("**make_calibrated_sim** (adapted evaluator, 20 s from reset; mm)\n\n"
                 "| Trajectory | PID RMSE | PID max | MPC RMSE | MPC max |\n|---|---|---|---|---|\n" + "\n".join(rows)))'''

SUMMARY = r"""## 11 — Summary and limitations

- **On the assessed plant (`make_sim`)** the MPC tracks the instructor's example to about 0.09 mm RMSE against
  1.26 mm for the template's PID, and beats the PID on every trajectory in §8. The gain comes from reading the
  supplied trajectory ahead: the arm needs about 0.6 s to respond to a pressure change, and the MPC commands that
  change before the reference moves.
- **Where both controllers are limited:** trajectories that start away from the tip (the tip is at the origin at
  reset) carry an unavoidable initial error, which dominates their RMSE and their maximum error; and fast paths
  (0.4 Hz circle) saturate the valves.
- **On `make_calibrated_sim`** the MPC still beats the PID, but both are dominated by the reach limit (x cannot
  exceed about +4.2 mm with S1 sealed) and the 0.5 s dead time.
- **Assumptions:** the reference function can be queried ahead of the current time, as the interface allows; the
  model parameters are identified in simulation (§3) and would need re-identifying on hardware; the tip velocity is
  a raw backward difference, which real sensor noise would amplify."""


M2SIM_EDITS = {   # name of the text block -> [(M3sim text, M2sim text), ...]
    "INTRO": [
        ("# Group 5 — Soft Robotic Arm: Model-Predictive Control with Trajectory Preview",
         "# Group 5 — Soft Robotic Arm: Model-Predictive Control with Trajectory Preview (M2 model)"),
        ("pneumatic lag $\\tau\\dot p = -p + Gu$ and second-order tip mechanics per axis.",
         "pneumatic lag $\\tau\\dot p = -p + Gu$ and first-order tip mechanics per axis (the structure of our model M2)."),
        ("RMSE; on our runs **0.09 mm RMSE**", "RMSE; on our runs **0.12 mm RMSE**"),
    ],
    "ID_MD": [
        ("""Per planar axis the tip follows its pressure input as a
  damped second-order system:
$$\\ddot y_i + 2\\zeta_i\\omega_i\\,\\dot y_i + \\omega_i^2\\,y_i = \\omega_i^2\\,(Hp)_i, \\qquad i \\in \\{x, y\\}$$""",
         """Per planar axis the tip follows its pressure input as a
  first-order lag (the structure of our model M2):
$$\\dot y_i = -a_i\\,y_i + a_i\\,(Hp)_i, \\qquad i \\in \\{x, y\\}$$"""),
        ("""A first-order mechanics model ($\\dot y_i = -a_i y_i + a_i (Hp)_i$) is fitted for comparison. The controller cell
(§5) contains these numbers, rounded.""",
         """A second-order mechanics model ($\\ddot y_i + 2\\zeta_i\\omega_i\\dot y_i + \\omega_i^2 y_i = \\omega_i^2 (Hp)_i$) is
fitted for comparison; it fits these data more closely (see the table), and a companion notebook uses it. The
controller cell (§5) contains the first-order numbers, rounded."""),
    ],
    "MPC_MD": [
        ("""$X = (y, \\dot y, p - 4.5) \\in \\mathbb R^8$: tip x-y (mm, measured), tip velocity (backward
difference of the measured tip), and the four measured segment pressures""",
         """$X = (y, p - 4.5) \\in \\mathbb R^6$: tip x-y (mm, measured) and the four measured segment pressures"""),
        ("$R_u = 0.01$ mm²/psi² is tuned in §7", "$R_u = 0.1$ mm²/psi² is tuned in §7"),
        ("""The controller keeps its last prediction, offset, previous correction and previous tip;""",
         """The controller keeps its last prediction, offset and previous correction;"""),
    ],
    "CONTROLLER": [
        ("""    "wn": np.array([7.081, 7.101]),                    # natural frequency, rad/s
    "zeta": np.array([0.699, 0.702]),                  # damping ratio
    "H": np.array([[1.794, -0.009, -1.792, 0.008],     # mm per psi of S1..S4, x row
                   [0.005, 1.794, 0.003, -1.803]]),    # y row""",
         """    "a": np.array([4.662, 4.694]),                     # rate of the first-order tip response, 1/s
    "H": np.array([[1.837, 0.011, -1.847, -0.002],     # mm per psi of S1..S4, x row
                   [0.004, 1.838, -0.001, -1.839]]),   # y row"""),
        ("""    def __init__(self, horizon=100, r_u=0.01, offset_tau_s=0.5):
        w, z, H, tau = MODEL["wn"], MODEL["zeta"], MODEL["H"], MODEL["tau_s"]
        O = np.zeros
        Ac = np.block([[O((2, 2)), np.eye(2), O((2, 4))],
                       [-np.diag(w ** 2), -np.diag(2 * z * w), np.diag(w ** 2) @ H],
                       [O((4, 4)), -np.eye(4) / tau]])
        Bc = np.vstack([O((4, 4)), np.diag(MODEL["gain"]) / tau]) @ PAIRS
        n, m, h = 8, 2, horizon""",
         """    def __init__(self, horizon=100, r_u=0.1, offset_tau_s=0.5):
        a, H, tau = MODEL["a"], MODEL["H"], MODEL["tau_s"]
        O = np.zeros
        Ac = np.block([[-np.diag(a), np.diag(a) @ H],
                       [O((4, 2)), -np.eye(4) / tau]])
        Bc = np.vstack([O((2, 4)), np.diag(MODEL["gain"]) / tau]) @ PAIRS
        n, m, h = 6, 2, horizon"""),
        ("""        self.C = np.hstack([np.eye(2), O((2, 6))])""", """        self.C = np.hstack([np.eye(2), O((2, 4))])"""),
        ("""        self.tip_prev, self.x_pred = None, None""", """        self.x_pred = None"""),
        ("""        vel = np.zeros(2) if self.tip_prev is None else (tip - self.tip_prev) / DT_S
        self.tip_prev = tip
        X = np.concatenate([tip, vel, np.asarray(obs["segment_pressures"], float) - BIAS_PSI])""",
         """        X = np.concatenate([tip, np.asarray(obs["segment_pressures"], float) - BIAS_PSI])"""),
    ],
    "TUNE_MD": [("$R_u = 0.01$, the value in the controller cell.", "$R_u = 0.1$, the value in the controller cell.")],
    "TUNE": [('("second order (submitted)"', '("first order, M2 (submitted)"')],
    "SUMMARY": [
        ("tracks the instructor's example to about 0.09 mm RMSE", "tracks the instructor's example to about 0.12 mm RMSE"),
        ("""  model parameters are identified in simulation (§3) and would need re-identifying on hardware; the tip velocity is
  a raw backward difference, which real sensor noise would amplify.""",
         """  model parameters are identified in simulation (§3) and would need re-identifying on hardware.
- **Model choice:** the second-order model of §3 fits `make_sim` more closely; the companion notebook
  `Soft_robotic_arm_MPC_M3sim.ipynb` runs the same MPC on it."""),
    ],
}


ONEMODE_BLOCKS = {   # Nikoletta's one-mode model (softarm.models.one_mode.reduced_model): y'' + c y' + k y = b dp
    "INTRO": r"""# Group 5 — Soft Robotic Arm: Model-Predictive Control on Nikoletta's One-Mode Model

Built on the course template (`soft_robotic_arm_v2.ipynb`). The assessment interface is unchanged:
the **controller cell** (§5) defines `student_controller(reference, obs)` and `reset_controller()`, and §6 runs
the instructor's `evaluate_controller` exactly as given.

**Controller.** The same model-predictive controller (MPC) as our other submissions, built on **Nikoletta's one-mode
model**: one bending mode per plane axis, $\ddot y + c\dot y + ky = b\,\Delta p$, with $k$, $c$, $b$ derived from the
arm's structure (joint springs, damping, lever arms), not fitted to data. Every 10 ms the MPC reads the supplied
trajectory one second ahead, `reference(t + 0.01 … t + 1.00)`, and computes the pressures that make the predicted tip
follow it, in closed form. Opposing segments act as pairs (S1/S3 for x, S2/S4 for y) around a 4.5 psi bias.

**Result on the instructor's example test** (§6, `make_sim(seed=2)`, 20 s, transient included): see the printed
RMSE; on our runs **0.95 mm RMSE**, against **1.26 mm** for the template's PID example. §8 compares the two on twelve
trajectories; §10 shows what changes when her model gets the gravity stiffness of the hanging arm and the measured
pneumatic lag.

Run all cells in order (Runtime → Run all), about 6 minutes on Colab. §9 (video) is optional and off by default.""",

    "ID_MD": r"""## 3 — The model: Nikoletta's one-mode model

Derived by Nikoletta from the MuJoCo arm (her notebook `SMC_Controller_withoutdisturbanceobserver.ipynb`, section 7):
the arm's bending joints are reduced to their dominant mode. Per plane axis $i \in \{x, y\}$:
$$\ddot y_i + c\,\dot y_i + k\,y_i = b\,\Delta p_i, \qquad \Delta p_x = p_{S1} - p_{S3}, \quad \Delta p_y = p_{S2} - p_{S4}$$
- $k$ is the mode's stiffness, from the joint springs; $c$ its damping, from the joint damping; $b$ the pressure gain
  through the lever arms. Derived, not identified.
- $\Delta p$ is the commanded pressure difference: the model has **no actuator dynamics** (the pouches are assumed to
  reach their commanded pressure at once) and no z.
- On `make_sim` all four segments are active, so x is driven by S1 − S3 and y by S2 − S4.

The cell below checks the model **free-running** against `make_sim` (random steps, seed 32, and a 0.02 → 1 Hz chirp),
as written and with two additions that are *not* in her notebook: the gravity stiffness of the hanging chain (same
derivation, $k$ from the springs plus gravity) and the pneumatic lag $\tau\dot p = -p + u$ with $\tau = 0.6$ s (the
value in the simulator's configuration, `make_sim().cfg.tau_pneumatic`, and measured in our other notebooks).""",

    "ID_CODE": r'''DT = 0.01
ONE_MODE = {"as written": (10.602415303139802, 9.230907320069726, 0.08203667215669287),
            "+ gravity stiffness": (51.45086866273482, 9.408628750381794, 0.07903934655742169)}   # k, c, |b| (m/psi)


def random_steps(seed, seconds=60.0):
    """Each segment jumps to an independent random level in 0-9 psi, held 0.5-3 s."""
    g = np.random.default_rng(seed)
    u, k = np.empty((int(seconds / DT), 4)), 0
    while k < len(u):
        n = int(g.uniform(0.5, 3.0) / DT)
        u[k:k + n] = g.uniform(0.0, 9.0, 4)
        k += n
    return u


def chirp(seconds=60.0):
    t = np.arange(int(seconds / DT)) * DT
    phase = 2 * np.pi * np.cumsum(0.02 * (1.0 / 0.02) ** (t / seconds)) * DT
    return 4.5 + 4.0 * np.column_stack([np.sin(phase), np.cos(phase), -np.sin(phase), -np.cos(phase)])


def run_open_loop(u, seed):
    """Tip x-y (mm) after each command step."""
    s = make_sim(control_hz=CONTROL_HZ, seed=seed)
    obs, tips = s.reset(), []
    for c in u:
        obs = s.step(c)
        tips.append(1000.0 * obs["tip_pos"][:2])
    s.close()
    return np.array(tips)


def free_run(u, k, c, b, lag):
    """One-mode model per axis from rest; dp = commanded (lag=False) or lagged (tau = 0.6 s) pressure difference."""
    from scipy.linalg import expm
    p = u.copy()
    if lag:
        a, prev = np.exp(-DT / 0.6), np.zeros(4)
        for i, c_ in enumerate(u):
            prev = a * prev + (1 - a) * c_
            p[i] = prev
    dp = np.column_stack([p[:, 0] - p[:, 2], p[:, 1] - p[:, 3]])
    E = expm(np.block([[np.array([[0.0, 1.0], [-k, -c]]), np.array([[0.0], [1000.0 * b]])], [np.zeros((1, 3))]]) * DT)
    A, B = E[:2, :2], E[:2, 2]
    out = np.zeros((len(u), 2))
    for i in range(2):
        x = np.zeros(2)
        for n, v in enumerate(dp[:, i]):
            x = A @ x + B * v
            out[n, i] = x[0]
    return out


rows = []
for name, u in (("random steps, seed 32", random_steps(32)), ("chirp 0.02 -> 1 Hz", chirp())):
    tips_v = run_open_loop(u, seed=3)
    for label, (k, c, b) in ONE_MODE.items():
        for lag in (False, True):
            e = free_run(u, k, c, b, lag) - tips_v
            rows.append(f"| {name} | {label}{' + pneumatic lag' if lag else ''} | {1000 * b / k:.2f} | "
                        f"{np.sqrt(np.mean(e[:, 0] ** 2)):.3f} | {np.sqrt(np.mean(e[:, 1] ** 2)):.3f} |")
display(Markdown("**Free-run check against make_sim** (RMSE, mm; the simulator's static gain is 1.81 mm per psi)\n\n"
                 "| Data | Model | static gain (mm/psi) | x | y |\n|---|---|---|---|---|\n" + "\n".join(rows)))''',

    "MPC_MD": r"""## 4 — The controller: MPC with trajectory preview

**State and input.** $X = (y, \dot y) \in \mathbb R^4$: tip x-y (mm, measured) and tip velocity (backward difference of
the measured tip). The input is a pair of differential corrections $\delta = (\delta_x, \delta_y)$: segment commands
$u = 4.5 + (\delta_x, \delta_y, -\delta_x, -\delta_y)$, so $\Delta p_x = 2\delta_x$ and $\Delta p_y = 2\delta_y$. The model of
§3 (as written: no actuator dynamics) is discretised with a zero-order hold at 10 ms: $X_{k+1} = AX_k + B\delta_k$,
tip $y = CX$.

**Offset.** The model is not exact. $e_k = y_k - C(AX_{k-1} + B\delta_{k-1})$ is its one-step prediction error;
its low-pass (0.5 s) is an offset $w$ added to every prediction:
$w_k = w_{k-1} + \alpha(e_k - w_{k-1})$, $\alpha = 1 - e^{-0.01/0.5}$.

**Optimisation.** Choose the next $H = 100$ corrections (1 s) to minimise
$$J = \sum_{i=0}^{H-1}\big\|C\hat X_{k+1+i} + w - r(t_k + (i+1)\,\Delta t)\big\|^2 + R_u\sum_{i=0}^{H-1}\big\|\delta_{k+i}-\delta_{k+i-1}\big\|^2$$
With $\hat Y = \Phi X_k + \Gamma U$ ($\Phi_i = CA^{i+1}$, $\Gamma_{ij} = CA^{i-j}B$ for $j \le i$, $D$ the
first-difference matrix), setting $\partial J/\partial U = 0$:
$$U^\star = \big(\Gamma^\top\Gamma + R_u D^\top D\big)^{-1}\Big[\Gamma^\top\big(\mathcal R - \Phi X_k - \mathbf 1\otimes w\big) + R_u D^\top e_1\,\delta_{k-1}\Big]$$
Only the first correction is applied: $\delta_k = g_r\,\mathcal R + g_x X_k + g_w\,w + g_u\,\delta_{k-1}$, then
$u = \operatorname{clip}_{[0,9]}\big(4.5 + (\delta_x, \delta_y, -\delta_x, -\delta_y)\big)$, sent to all five pouches of each
segment. $R_u = 1$ mm²/psi² is tuned in §7 on trajectories not used for testing.

The class also takes `gravity=True` (her derivation with the hanging chain's gravity stiffness) and
`pneumatic_lag=True` (adds the four measured segment pressures to the state, with $\tau\dot p = -p + u$); §10 uses
them. The submitted controller uses her model as written.

**Memory.** The controller keeps its last prediction, offset, previous correction and previous tip;
`reset_controller()` clears them before each trial.""",

    "CONTROLLER": r'''# STUDENT CONTROLLER CELL: MPC with trajectory preview on Nikoletta's one-mode model (Group 5).
import numpy as np
from scipy.linalg import expm

# Nikoletta's one-mode model per plane axis: y'' + c y' + k y = b dp  (y in m, dp in psi), derived from the arm.
ONE_MODE_PARAMS = {False: dict(k=10.602415303139802, c=9.230907320069726, b=0.08203667215669287),   # as written
                   True: dict(k=51.45086866273482, c=9.408628750381794, b=0.07903934655742169)}    # + gravity
BIAS_PSI, LIMIT_PSI, DT_S, TAU_S = 4.5, 9.0, 0.01, 0.6
PAIRS = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]], float)   # u = BIAS + PAIRS @ (dx, dy)


class PreviewMPC:
    """Unconstrained MPC on the one-mode model, reading the supplied trajectory HORIZON steps ahead."""

    def __init__(self, horizon=100, r_u=1.0, offset_tau_s=0.5, gravity=False, pneumatic_lag=False):
        prm = ONE_MODE_PARAMS[gravity]
        k, c, b = prm["k"], prm["c"], 1000.0 * prm["b"]          # b in mm per psi of dp
        O, I = np.zeros, np.eye(2)
        if pneumatic_lag:   # state (y, y', p - BIAS): dp = (p1 - p3, p2 - p4), tau p' = -p + u
            Hp = b * np.array([[1, 0, -1, 0], [0, 1, 0, -1]], float)
            Ac = np.block([[O((2, 2)), I, O((2, 4))], [-k * I, -c * I, Hp], [O((4, 4)), -np.eye(4) / TAU_S]])
            Bc = np.vstack([O((4, 4)), np.eye(4) / TAU_S]) @ PAIRS
        else:               # as written: dp = commanded difference = 2 * delta, no actuator dynamics
            Ac = np.block([[O((2, 2)), I], [-k * I, -c * I]])
            Bc = np.vstack([O((2, 2)), 2 * b * I])
        n, m, h = Ac.shape[0], 2, horizon
        self.lag = pneumatic_lag
        E = expm(np.block([[Ac, Bc], [O((m, n + m))]]) * DT_S)
        self.A, self.B = E[:n, :n], E[:n, n:]
        self.C = np.hstack([I, O((2, n - 2))])
        powers = [np.eye(n)]
        for _ in range(h):
            powers.append(self.A @ powers[-1])
        Phi = np.vstack([self.C @ powers[i + 1] for i in range(h)])
        Gam = np.zeros((2 * h, m * h))
        for i in range(h):
            for j in range(i + 1):
                Gam[2 * i:2 * i + 2, m * j:m * j + m] = self.C @ powers[i - j] @ self.B
        D = np.eye(m * h) - np.eye(m * h, k=-m)
        K = np.linalg.solve(Gam.T @ Gam + r_u * D.T @ D, np.eye(m * h))[:m]   # first move only
        self.g_r = K @ Gam.T
        self.g_x = -self.g_r @ Phi
        self.g_w = -self.g_r @ np.tile(np.eye(2), (h, 1))
        E1 = np.zeros((m * h, m))
        E1[:m] = np.eye(m)
        self.g_u = r_u * K @ D.T @ E1
        self.steps_ahead = (np.arange(h) + 1) * DT_S
        self.alpha = 1.0 - np.exp(-DT_S / offset_tau_s)
        self.reset()

    def reset(self):
        self.tip_prev, self.x_pred = None, None
        self.offset, self.delta_prev = np.zeros(2), np.zeros(2)

    def __call__(self, reference, obs):
        t = float(obs["time"])
        tip = 1000.0 * np.asarray(obs["tip_pos"], float)[:2]                     # mm
        vel = np.zeros(2) if self.tip_prev is None else (tip - self.tip_prev) / DT_S
        self.tip_prev = tip
        parts = [tip, vel] + ([np.asarray(obs["segment_pressures"], float) - BIAS_PSI] if self.lag else [])
        X = np.concatenate(parts)
        if self.x_pred is not None:                                              # model offset
            self.offset += self.alpha * ((tip - self.C @ self.x_pred) - self.offset)
        window = 1000.0 * np.array([reference(t + s) for s in self.steps_ahead])  # mm, (H, 2)
        delta = (self.g_r @ window.reshape(-1) + self.g_x @ X
                 + self.g_w @ self.offset + self.g_u @ self.delta_prev)
        segments = np.clip(BIAS_PSI + PAIRS @ delta, 0.0, LIMIT_PSI)
        applied = np.array([segments[0] - segments[2], segments[1] - segments[3]]) / 2.0
        self.x_pred = self.A @ X + self.B @ applied
        self.delta_prev = applied
        return np.repeat(segments[:, None], 5, axis=1)                          # (4, 5) pouch psi


_mpc = PreviewMPC()   # Nikoletta's model as written


def reset_controller():
    """Clear all controller memory before a new trial."""
    _mpc.reset()


def student_controller(reference, obs):
    """Track the supplied x-y trajectory; return absolute (4, 5) pouch pressures in psi."""
    return _mpc(reference, obs)


reset_controller()''',

    "TUNE_MD": r"""## 7 — Tuning $R_u$ on separate trajectories

$R_u$ trades tracking against smooth commands. It is chosen on three trajectories that are **not** used for
testing: the template's tutorial family with seed 11 (12 mm), a smooth random path (seed 12) and random waypoints
(seed 13). Score: mean RMSE over the three with the instructor's `evaluate_controller`. The smallest mean is at
$R_u = 1$, the value in the controller cell. Our identified models tune to much smaller $R_u$; this model's static
gain is about 4× the simulator's (§3), and the larger $R_u$ damps the commands it produces.""",

    "TUNE": r'''TUNING = {"tutorial sines, seed 11": tutorial_sines(11), "smooth random, seed 12": smooth_random(12),
          "waypoints, seed 13": waypoints(13)}
grid = (1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0)
rows = []
for r_u in grid:
    scores = []
    for ref in TUNING.values():
        ctl = PreviewMPC(r_u=r_u)
        scores.append(evaluate_controller(ctl, ref, reset=ctl.reset)["rmse_mm"])
    rows.append(f"| one-mode, as written (submitted) | {r_u:g} | " + " | ".join(f"{s:.3f}" for s in scores)
                + f" | **{np.mean(scores):.3f}** |")
display(Markdown("| Model | R_u | " + " | ".join(TUNING) + " | mean |\n|---|---|---|---|---|---|\n" + "\n".join(rows)))''',

    "SUMMARY": r"""## 11 — Summary and limitations

- **Her model as written** already gives a working tracking controller: about 0.95 mm RMSE on the instructor's example
  against 1.26 mm for the template's PID (§6, §8). The gain comes from reading the supplied trajectory ahead.
- **Its two gaps on `make_sim`** (§3, §10): the static gain is about 4× the simulator's, because the gravity stiffness
  of the hanging arm is left out, and the commanded pressure is assumed to act at once, while the pouches lag by
  0.6 s. With both added, the same MPC reaches about 0.17 mm on the instructor's example; the same structure fitted to
  data (our M3sim notebook) reaches 0.09 mm.
- **Where all controllers are limited:** trajectories that start away from the tip carry an unavoidable initial error,
  and fast paths (0.4 Hz circle) saturate the valves.
- **Assumptions:** the reference function can be queried ahead of the current time, as the interface allows; the tip
  velocity is a raw backward difference, which real sensor noise would amplify.""",
}

VARIANTS_MD = r"""## 10 — Variants of her model: gravity stiffness and pneumatic lag

The same MPC on four versions of the one-mode model, each with its own $R_u$ tuned on the §7 trajectories (grid
$10^{-4}$ to $10$), then run on all twelve test trajectories. *As written* is the submitted controller. *+ gravity*
uses her derivation with the hanging chain's gravity stiffness; *+ lag* adds the measured 0.6 s pneumatic lag to the
model's state. Neither addition is in her notebook. Each cell: RMSE in mm (saturation, % of pouch commands at 0 or
9 psi)."""

VARIANTS_CODE = r'''VARIANTS = {"as written": dict(), "+ gravity": dict(gravity=True), "+ lag": dict(pneumatic_lag=True),
            "+ gravity + lag": dict(gravity=True, pneumatic_lag=True)}
best_r_u = {}
for name, kw in VARIANTS.items():
    means = {}
    for r_u in (1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0):
        scores = []
        for ref in TUNING.values():
            ctl = PreviewMPC(r_u=r_u, **kw)
            scores.append(evaluate_controller(ctl, ref, reset=ctl.reset)["rmse_mm"])
        means[r_u] = np.mean(scores)
    best_r_u[name] = min(means, key=means.get)
print("tuned R_u:", best_r_u)

head = "| Trajectory | " + " | ".join(f"{n} (R_u {best_r_u[n]:g})" for n in VARIANTS) + " |"
rows = []
for test, ref in TESTS.items():
    cells = []
    for name, kw in VARIANTS.items():
        ctl = PreviewMPC(r_u=best_r_u[name], **kw)
        res = evaluate_controller(ctl, ref, reset=ctl.reset)
        cells.append(f"{res['rmse_mm']:.3f} ({saturation_pct(res):.0f} %)")
    rows.append(f"| {test} | " + " | ".join(cells) + " |")
display(Markdown(head + "\n|" + "---|" * (len(VARIANTS) + 1) + "\n" + "\n".join(rows)))'''


def text(name: str, variant: str) -> str:
    if variant == "OneMode" and name in ONEMODE_BLOCKS:
        return ONEMODE_BLOCKS[name]
    out = globals()[name]
    for old, new in (M2SIM_EDITS.get(name, []) if variant == "M2sim" else []):
        assert out.count(old) == 1, (name, old[:60])
        out = out.replace(old, new)
    return out


def build(variant: str) -> nbf.NotebookNode:
    T = template_cells()
    t = lambda name: text(name, variant)  # noqa: E731
    cells = [md(t("INTRO")),
             md(T[1]), code(T[2]),                    # 1 - install (template)
             md("## 2 — Simulator setup (template)"), code(T[4] + "\n\nfrom IPython.display import Markdown"),
             md(t("ID_MD")), code(t("ID_CODE")),
             md(t("MPC_MD")),
             md("## 5 — Controller cell\n\nSelf-contained: it runs on its own after the setup cell."), code(t("CONTROLLER")),
             md("## 6 — Instructor evaluation (template, unchanged)\n\n" + T[15].split("\n", 1)[1]),
             code(T[17]), code(T[18]), md(T[19]), code(T[20]),
             md(t("TUNE_MD")), code(TRAJ), code(t("TUNE")),
             md(COMPARE_MD), code(COMPARE), code(OVERLAY),
             md("## 9 — Render the controller simulation video (optional, template)\n\n"
                "Set `RENDER_VIDEO = True` to record our controller on the instructor's example (needs a GPU-less EGL "
                "renderer, available on Colab)."),
             code("RENDER_VIDEO = False\nif RENDER_VIDEO:\n" + "\n".join(
                 "    " + line for line in T[14]
                 .replace("video_controller = RandomTrajectoryPDController()", "reset_controller()")
                 .replace("video_controller.compute(video_obs[\"time\"], video_obs)",
                          "student_controller(instructor_reference, video_obs)")
                 .replace("soft_arm_pd_tracking.mp4", "group5_mpc_tracking.mp4").splitlines())),
             *([md(VARIANTS_MD), code(VARIANTS_CODE)] if variant == "OneMode" else [md(CALIB_MD), code(CALIB_CODE)]),
             md(t("SUMMARY"))]
    return nbf.v4.new_notebook(cells=cells, metadata={
        "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"}, "colab": {"provenance": []}})


if __name__ == "__main__":
    for variant, path in OUT.items():
        nbf.write(build(variant), path)
        print(f"wrote {path.relative_to(ROOT)}")
