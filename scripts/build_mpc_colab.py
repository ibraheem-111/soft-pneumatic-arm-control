"""Write the standalone Colab submission notebooks for the MPC on M2 and the MPC on M3.

    notebooks/Soft_robotic_arm_MPC_M2.ipynb
    notebooks/Soft_robotic_arm_MPC_M3.ipynb

Each runs on Google Colab (or any Python >= 3.10) with no files from this repository: it installs
soft-robotic-arm 0.4.2, defines the model with its fitted parameters (copied from
docs/figures/task_space_model.json), the controller, the tests and the plots. The controller is the
one in scripts/compare_m2_controllers.py (M2MPC) and scripts/mpc_m3.py; the tests and metrics are
those of compare_m2_controllers.py. Both notebooks share every cell except the model section.

Usage:
    uv run python scripts/build_mpc_colab.py
    uv run jupyter nbconvert --to notebook --execute notebooks/Soft_robotic_arm_MPC_M2.ipynb --output /tmp/check.ipynb
"""

import json
import sys
from pathlib import Path

import nbformat as nbf

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ndosmc as N  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
md, code = nbf.v4.new_markdown_cell, nbf.v4.new_code_cell


def params() -> tuple[dict, dict, dict]:
    m = N.TaskSpaceModel.load()
    act = dict(gain=[float(g) for g in m.actuator.gain], tau_s=float(m.actuator.tau_s),
               delay_steps=int(round(m.actuator.delay_s * 100)))
    m2 = dict(a=[float(ax.params(0.0)[0]) for ax in m.axes_measured], H=m.B_measured.tolist())
    mech = json.loads(N.MODEL_JSON.read_text())["M3_mechanics"]
    m3 = dict(wn=[mech[a]["wn"] for a in "xyz"], zeta=[mech[a]["zeta"] for a in "xyz"], H=[mech[a]["H"] for a in "xyz"])
    return act, m2, m3


# ------------------------------------------------------------------------------------ shared cells

INSTALL = '''import importlib.util, subprocess, sys

NEEDED = {"soft_robotic_arm": "soft-robotic-arm==0.4.2", "scipy": "scipy"}   # Colab already has scipy
missing = [pkg for mod, pkg in NEEDED.items() if importlib.util.find_spec(mod) is None]
if missing:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-q", *missing])
import soft_robotic_arm
print("soft-robotic-arm", getattr(soft_robotic_arm, "__version__", "installed"))'''

IMPORTS = '''import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm
from IPython.display import Markdown, display
from soft_robotic_arm import make_calibrated_sim
from soft_robotic_arm.evaluation import CircularTrackingTask, evaluate_controller, make_reference

HZ, DT = 100.0, 0.01          # control rate (Hz) and step (s)
BASE, LIMIT = 4.5, 9.0        # settled command and command limit, psi
SETTLE_S, TRACK_S = 5.0, 30.0 # as CircularTrackingTask: settle at BASE, then track

# Plot style (colour-blind safe, Okabe-Ito): every series has its own colour, line style and marker.
BLACK, ORANGE, SKY, BLUE, VERMILLION, PURPLE = "#000000", "#E69F00", "#56B4E9", "#0072B2", "#D55E00", "#CC79A7"
STYLE = {"S2": (ORANGE, "-", "o"), "S3": (SKY, "--", "s"), "S4": (PURPLE, "-.", "^"),
         "x": (BLUE, "-", "o"), "y": (VERMILLION, "--", "s"),
         "e x": (BLUE, ":", "o"), "e y": (VERMILLION, ":", "s"), "w x": (BLUE, "-", "^"), "w y": (VERMILLION, "-", "D")}
REF = dict(color=BLACK, ls=(0, (6, 3)), lw=3, label="reference")
plt.rcParams.update({"font.size": 13, "axes.titlesize": 14, "axes.labelsize": 13, "legend.fontsize": 12,
                     "lines.linewidth": 2.2, "figure.dpi": 110})


def line(name, n, **kw):
    c, ls, mk = STYLE[name]
    return {**dict(color=c, ls=ls, marker=mk, markevery=max(1, n // 8), markersize=7, label=name), **kw}


def md_table(head, rows):
    out = ["| " + " | ".join(head) + " |", "|---" * len(head) + "|"]
    display(Markdown("\\n".join(out + ["| " + " | ".join(str(c) for c in r) + " |" for r in rows])))'''

MPC_MD = r"""## 4 — The controller: delay-compensated MPC with the trajectory known in advance

Every 10 ms step $k$ (state $X$, input $u$ = command − 4.5 psi, tip $y = CX$ in mm from the settled pose):

**1. Discretise** the model with a zero-order hold: $A = e^{A_c\Delta t}$, $B = \int_0^{\Delta t} e^{A_c s}B_c\,ds$, so
$X_{k+1} = A X_k + B u_{k-N}$ with dead time $N = 50$ steps (0.5 s).

**2. Predict across the dead time** with the commands already sent (they cannot be changed):
$$\hat X_{k+N} = A^N X_k + \sum_{i=0}^{N-1} A^{N-1-i} B\,u_{k-N+i}$$

**3. Offset.** The model is not exact. $e_k = y_k - C\hat X_{k\,|\,k-N}$ is the tip error of the prediction made $N$
steps ago; its low-pass (0.5 s) is the offset $w$, held constant over the horizon:
$$w_k = w_{k-1} + \alpha\,(e_k - w_{k-1}), \qquad \alpha = 1 - e^{-\Delta t/0.5\,\mathrm s}$$

**4. Optimise** the next $H$ commands against the known reference $r_{k+N}\dots r_{k+N+H-1}$:
$$J = \sum_{i=0}^{H-1}\big\|C\hat X_{k+N+1+i} + w - r_{k+N+i}\big\|^2 + R_u\sum_{i=0}^{H-1}\big\|u_{k+i}-u_{k+i-1}\big\|^2$$
Stacking $\hat Y = \Phi\hat X_{k+N} + \Gamma U$ ($\Phi_i = CA^{i+1}$, $\Gamma_{ij} = CA^{i-j}B$ for $j\le i$; $D$ the
first-difference matrix), $\partial J/\partial U = 0$ gives
$$U^\star = \big(\Gamma^\top\Gamma + R_u D^\top D\big)^{-1}\Big[\Gamma^\top\big(\mathcal R - \Phi\hat X_{k+N} - \mathbf 1\otimes w\big) + R_u D^\top e_1 u_{k-1}\Big]$$

**5. Apply the first move only**, a fixed linear law computed once:
$$u_k = g_r\,\mathcal R + g_x\,\hat X_{k+N} + g_w\,w + g_u\,u_{k-1}, \qquad \text{command} = \operatorname{clip}_{[0,\,9]}(4.5 + u_k)$$
The clipped command is what enters the prediction buffer. $g_r$ is a feedforward over the whole reference window, so
the controller starts turning before a corner reaches the arm.

**Interface.** `compute(t, obs, ref)` returns the absolute S2–S4 pressures, using only the tip position and the pouch
pressures. The professor confirmed the test trajectory may be assumed known, so the controller takes it as a function
`trajectory(t)` of tracking time (offset from the settled tip, metres) and reads its window ahead. With
`trajectory=None` it falls back to holding the current reference over the window (no preview)."""

CONTROLLER = '''class DelayMPC:
    """Delay-compensated unconstrained MPC on MODEL_NAME; compute(t, obs, ref) -> S2-S4 psi."""

    def __init__(self, trajectory=None, horizon=HORIZON, r_u=R_U, offset_tau_s=0.5, max_track_s=60.0, log=False):
        Ac, Bc, C = model_matrices()
        n = Ac.shape[0]
        E = expm(np.block([[Ac, Bc], [np.zeros((3, n + 3))]]) * DT)
        A, B = E[:n, :n], E[:n, n:]
        self.C, self.n, self.h, self.nd = C, n, horizon, ACT["delay_steps"]
        self.alpha = 1.0 - np.exp(-DT / offset_tau_s)
        h, nd = horizon, self.nd
        Apow = [np.eye(n)]
        for _ in range(max(h, nd) + 1):
            Apow.append(A @ Apow[-1])
        Phi = np.vstack([C @ Apow[i + 1] for i in range(h)])
        Gam = np.zeros((3 * h, 3 * h))
        for i in range(h):
            for j in range(i + 1):
                Gam[3 * i:3 * i + 3, 3 * j:3 * j + 3] = C @ Apow[i - j] @ B
        D = np.eye(3 * h) - np.eye(3 * h, k=-3)
        K = np.linalg.inv(Gam.T @ Gam + r_u * D.T @ D)[:3]            # first move only
        self.g_r = K @ Gam.T
        self.g_x = -self.g_r @ Phi
        self.g_w = -self.g_r @ np.tile(np.eye(3), (h, 1))
        E1 = np.zeros((3 * h, 3))
        E1[:3] = np.eye(3)
        self.g_u = r_u * K @ D.T @ E1
        self.AN = Apow[nd]
        self.W = np.hstack([Apow[nd - 1 - i] @ B for i in range(nd)])   # (n, 3N): acts on u(k-N) ... u(k-1)
        self.R = None
        if trajectory is not None:                                      # known reference, mm, sampled once
            m = int(round(max_track_s * HZ)) + nd + h + 1
            self.R = np.array([1000.0 * np.asarray(trajectory(i * DT), float) for i in range(m)])
        self.k, self.home, self.p0, self.y_prev = 0, None, None, None
        self.buf = np.zeros((nd, 3))
        self.u_prev, self.w = np.zeros(3), np.zeros(3)
        self.pred = []
        self.log = {key: [] for key in ("p", "e", "w", "E")} if log else None

    def compute(self, t, obs, ref):
        tip = np.asarray(obs["tip_pos"], float)
        seg = np.asarray(obs["segment_pressures"], float)[1:4]
        if self.home is None:                                           # first call: the settled pose
            self.home, self.p0 = tip.copy(), seg.copy()
        y = 1000.0 * (tip - self.home)
        X = measure_state(y, self.y_prev, seg - self.p0)
        self.y_prev = y
        Xf = self.AN @ X + self.W @ self.buf.reshape(-1)                # step 2
        self.pred.append(self.C @ Xf)
        e = np.zeros(3)
        if len(self.pred) > self.nd:                                    # step 3
            e = y - self.pred.pop(0)
            self.w += self.alpha * (e - self.w)
        if self.R is not None:                                          # r(k+N) ... r(k+N+H-1)
            idx = np.minimum(self.k + self.nd + np.arange(self.h), len(self.R) - 1)
            r = self.R[idx]
        else:
            r = np.tile(1000.0 * (np.asarray(ref, float) - self.home), (self.h, 1))
        u = self.g_r @ r.reshape(-1) + self.g_x @ Xf + self.g_w @ self.w + self.g_u @ self.u_prev   # step 5
        cmd = np.clip(BASE + u, 0.0, LIMIT)
        u_app = cmd - BASE
        self.buf = np.vstack([self.buf[1:], u_app])
        self.u_prev = u_app
        if self.log is not None:
            for key, val in (("p", seg), ("e", e), ("w", self.w.copy()), ("E", self.C @ Xf + self.w - r[0])):
                self.log[key].append(val)
        self.k += 1
        return cmd'''

TESTS_MD = r"""## 5 — Tests and metrics

`track()` is `evaluate_controller`'s loop (settle 5 s at 4.5 psi, then 30 s at 100 Hz; the reference is centred on the
settled tip) for any known trajectory. The circles are also run through the package's own `evaluate_controller` in §7
to show the two agree.

| Test | Reference | Sim seed | "After settling" starts |
|---|---|---|---|
| scored circle | 7 mm, 0.1 Hz (`CircularTrackingTask()`) | 0 | after the first lap (10 s) |
| held-out circle | 10 mm, 0.15 Hz | 5 | after the first lap (6.7 s) |
| triangle | vertices on a 7 mm circle, first at +x, 10 s per lap | 0 | after the first lap |
| square | axis-aligned, vertices on a 7 mm circle (side 9.9 mm), 10 s per lap | 0 | after the first lap |
| squiggle | per axis 6 random sines, 0.03–0.3 Hz, ±6 mm, path seed 101 | 0 | 5 s |
| random waypoints | straight lines between random points, ±6 mm, 1–2.5 s each, path seed 101 | 0 | 5 s |

Metrics, planar x–y, mm: **RMSE whole run** (what the harness reports); **RMSE after settling**; **shape error** (RMS
distance from the tip to the nearest point of the reference path, ignoring timing); **max error** after settling;
**radius range** (circles) or **path size** (triangle, square: 100 % = right size); **saturation** (% of commands at 0 or
9 psi). The controller's $R_u$ is tuned in §6 on a separate circle, never on a test."""

TESTS = '''def circle(radius_m, freq):
    task = CircularTrackingTask(radius_m=radius_m, frequency_hz=freq)
    return lambda t: make_reference(np.zeros(3), t, task)


def polygon(n, radius_m, lap_s, first_deg=0.0):
    th = np.radians(first_deg) + 2 * np.pi * np.arange(n + 1) / n
    v = radius_m * np.column_stack([np.cos(th), np.sin(th)])

    def fn(t):
        s = (t / lap_s % 1.0) * n
        k = int(s)
        return np.append(v[k] + (s - k) * (v[k + 1] - v[k]), 0.0)
    return fn


def smooth_random(seed, span_mm=6.0, f_lo=0.03, f_hi=0.3, n=6):
    g = np.random.default_rng(seed)
    f, ph, a = g.uniform(f_lo, f_hi, (2, n)), g.uniform(0, 2 * np.pi, (2, n)), g.uniform(0.3, 1.0, (2, n))
    raw = lambda t: (a * np.sin(2 * np.pi * f * np.asarray(t)[..., None, None] + ph)).sum(-1)
    scale = span_mm / 1000.0 / np.abs(raw(np.arange(0.0, TRACK_S, DT)) - raw(0.0)).max(axis=0)
    return lambda t: np.append((raw(t) - raw(0.0)) * scale, 0.0)


def random_waypoints(seed, span_mm=6.0, seg_lo=1.0, seg_hi=2.5):
    g = np.random.default_rng(seed)
    times, pts = [0.0], [np.zeros(2)]
    while times[-1] < TRACK_S + 10.0:
        times.append(times[-1] + g.uniform(seg_lo, seg_hi))
        pts.append(g.uniform(-span_mm, span_mm, 2) / 1000.0)
    pts = np.array(pts)
    return lambda t: np.array([np.interp(t, times, pts[:, 0]), np.interp(t, times, pts[:, 1]), 0.0])


TESTS = {  # name: (trajectory(t) -> offset from the settled tip in m, sim seed, settle_s, kind)
    "scored circle": (circle(0.007, 0.10), 0, 10.0, "circle"),
    "held-out circle": (circle(0.010, 0.15), 5, 1 / 0.15, "circle"),
    "triangle": (polygon(3, 0.007, 10.0), 0, 10.0, "polygon"),
    "square": (polygon(4, 0.007, 10.0, 45.0), 0, 10.0, "polygon"),
    "squiggle": (smooth_random(101), 0, 5.0, "open"),
    "random waypoints": (random_waypoints(101), 0, 5.0, "open"),
}


def track(controller, trajectory, seed=0, track_s=TRACK_S):
    """evaluate_controller's loop for any known trajectory. Returns tip, reference (mm from home) and commands."""
    sim = make_calibrated_sim(control_hz=HZ, seed=seed)
    obs = sim.reset()
    for _ in range(int(round(SETTLE_S * HZ))):
        obs = sim.step(np.full(3, BASE))
    home = np.asarray(obs["tip_pos"], float).copy()
    n = int(round(track_s * HZ))
    tips, refs, cmds = np.empty((n, 3)), np.empty((n, 3)), np.empty((n, 3))
    for k in range(n):
        ref = home + np.asarray(trajectory(k * DT), float)
        cmd = np.clip(np.asarray(controller.compute(float(obs["time"]), obs, ref), float), 0.0, LIMIT)
        obs = sim.step(cmd)
        tips[k], refs[k], cmds[k] = obs["tip_pos"], ref, cmd
    sim.close()
    return dict(tip=1000.0 * (tips - home), ref=1000.0 * (refs - home), cmd=cmds)


def score(run, settle_s, kind):
    tip, ref, cmds = run["tip"], run["ref"], run["cmd"]
    e = np.linalg.norm(tip[:, :2] - ref[:, :2], axis=1)
    k0 = int(round(settle_s * HZ))
    path = ref[k0:, :2] if kind == "open" else ref[-k0:, :2]
    dense = np.vstack([np.linspace(a, b, 5, endpoint=False) for a, b in zip(path[:-1], path[1:])])
    t = tip[k0:, :2]
    near = np.concatenate([np.sqrt(((t[i:i + 300, None] - dense[None]) ** 2).sum(-1)).min(1) for i in range(0, len(t), 300)])
    out = dict(rmse_whole=np.sqrt(np.mean(e ** 2)), rmse_settled=np.sqrt(np.mean(e[k0:] ** 2)),
               shape=np.sqrt(np.mean(near ** 2)), max_settled=e[k0:].max(),
               saturated_pct=100 * np.mean((cmds <= 0.0) | (cmds >= LIMIT)))
    if kind == "circle":
        rad = np.linalg.norm(t - t.mean(0), axis=1)
        out["radius"] = (rad.min(), rad.max(), np.linalg.norm(ref[0, :2] - ref[:, :2].mean(0)))
    if kind == "polygon":
        rms = lambda a: np.sqrt(np.mean(np.sum((a - a.mean(0)) ** 2, 1)))
        out["size_pct"] = 100 * rms(t) / rms(ref[k0:, :2])
    return out'''

TUNE_MD = r"""## 6 — Tuning $R_u$ (on a separate circle, never on a test)

$R_u$ trades tracking against smooth commands. It is swept on a 5 mm, 0.07 Hz circle with sim seed 5, a task used for
nothing else; the score is RMSE after the first lap. The smallest weight wins, so the controller is close to an inverse
of the model; larger weights cost accuracy on smooth circles."""

TUNE = '''tune_traj = circle(0.005, 0.07)
grid = (1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0)
tune_rmse = []
for r_u in grid:
    run = track(DelayMPC(tune_traj, r_u=r_u), tune_traj, seed=5)
    tune_rmse.append(score(run, 1 / 0.07, "circle")["rmse_settled"])
md_table(["R_u (mm²/psi²)"] + [f"{g:g}" for g in grid], [["RMSE after first lap (mm)"] + [f"{e:.4f}" for e in tune_rmse]])
print(f"best R_u = {grid[int(np.argmin(tune_rmse))]:g}; the controller uses R_U = {R_U:g}")'''

OFFICIAL_MD = """## 7 — Official harness: `evaluate_controller` on the scored task and the held-out circle

The scored number is the package's `evaluate_controller` with `CircularTrackingTask()` on `make_calibrated_sim(seed=0)`:
RMSE over the whole 30 s, which includes the first 0.5 s before any command can reach the arm and the jump to the
circle's start point. The held-out circle (10 mm, 0.15 Hz, seed 5) checks the controller off the scored trajectory."""

OFFICIAL = '''official = {}
for name, task, seed in (("scored circle", CircularTrackingTask(), 0),
                         ("held-out circle", CircularTrackingTask(radius_m=0.010, frequency_hz=0.15), 5)):
    traj = lambda t, task=task: make_reference(np.zeros(3), t, task)
    res = evaluate_controller(DelayMPC(traj), task=task, sim=make_calibrated_sim(control_hz=task.control_hz, seed=seed))
    official[name] = res
md_table(["Task", "RMSE (mm)", "max error (mm)", "phase lag (s)", "effort (psi)"],
         [[n, f"{r.rmse_mm:.3f}", f"{r.max_error_mm:.2f}", f"{r.phase_lag_s:+.3f}", f"{r.effort_psi:.2f}"]
          for n, r in official.items()])'''

RUN_MD = """## 8 — All six tests

Each run logs the controller's internals for the plots in §9. The two circles reproduce the official RMSE above."""

RUN = '''runs, logs, scores = {}, {}, {}
for name, (traj, seed, settle_s, kind) in TESTS.items():
    ctl = DelayMPC(traj, log=True)
    runs[name] = track(ctl, traj, seed=seed)
    logs[name] = {k: np.array(v) for k, v in ctl.log.items()}
    scores[name] = score(runs[name], settle_s, kind)

rows = []
for name, s in scores.items():
    kind = TESTS[name][3]
    extra = (f"radius {s['radius'][0]:.1f}–{s['radius'][1]:.1f} (ref {s['radius'][2]:.1f})" if kind == "circle"
             else f"size {s['size_pct']:.0f} %" if kind == "polygon" else "")
    rows.append([name, f"{s['rmse_whole']:.3f}", f"{s['rmse_settled']:.3f}", f"{s['shape']:.3f}",
                 f"{s['max_settled']:.2f}", f"{s['saturated_pct']:.1f}", extra])
md_table(["Test", "RMSE whole run", "RMSE after settling", "Shape error", "Max after settling", "Saturation %", "Size"], rows)'''

SEEDS_MD = """### Random trajectories, three seeds each

The final test uses random, irregular trajectories, so the two random families are also run on path seeds 101–103."""

SEEDS = '''rows = []
for fam, make in (("squiggle", smooth_random), ("random waypoints", random_waypoints)):
    for seed in (101, 102, 103):
        traj = make(seed)
        s = score(track(DelayMPC(traj), traj), 5.0, "open")
        rows.append([fam, seed, f"{s['rmse_whole']:.3f}", f"{s['rmse_settled']:.3f}", f"{s['shape']:.3f}",
                     f"{s['max_settled']:.2f}", f"{s['saturated_pct']:.1f}"])
md_table(["Trajectory", "Path seed", "RMSE whole run", "RMSE after 5 s", "Shape error", "Max after 5 s", "Saturation %"], rows)'''

PLOTS_MD = r"""## 9 — Plots for every test

For each test:
1. **Tracking:** the tip path against the reference (last lap, or after 5 s) and the planar error over time.
2. **Inside the controller:** commanded pressure $u$; measured segment pressure $p$; the disturbance as the model sees it,
   $e = y - C\hat X_{k|k-N}$ (the tip error of the 0.5 s-ahead prediction, dotted), with the offset $w$ that the
   controller subtracts (its 0.5 s low-pass, solid).
3. **Error-space phase portrait:** $E = C\hat X_{k+N} + w - r_{k+N}$, the error the MPC aims to remove first, against
   $\dot E$. Left: whole run (the start-up jump being closed). Right: after settling. There is no sliding line: the MPC
   has no switching surface."""

PLOTS = '''def show(name):
    traj, seed, settle_s, kind = TESTS[name]
    run, log = runs[name], logs[name]
    n = len(run["tip"])
    t = np.arange(n) * DT
    k0 = int(round(settle_s * HZ))
    sl = slice(k0, None) if kind == "open" else slice(n - k0, None)

    fig, axs = plt.subplots(1, 2, figsize=(14, 5.5), layout="constrained")
    axs[0].plot(run["ref"][sl, 0], run["ref"][sl, 1], **REF)
    axs[0].plot(run["tip"][sl, 0], run["tip"][sl, 1], **line("x", n - sl.start, label=f"tip, {MODEL_NAME} MPC"))
    axs[0].set(xlabel="x − home (mm)", ylabel="y − home (mm)", title=f"tip path, {'after 5 s' if kind == 'open' else 'last lap'}")
    axs[0].set_aspect("equal", adjustable="datalim")
    e = np.linalg.norm(run["tip"][:, :2] - run["ref"][:, :2], axis=1)
    axs[1].plot(t, e, **line("x", n, label=f"RMSE after settling {scores[name]['rmse_settled']:.3f} mm"))
    axs[1].set(xlabel="tracking time (s)", ylabel="planar error (mm)", title="planar error", ylim=(0, None))
    for a in axs:
        a.grid(alpha=0.3)
        a.legend(loc="upper right")
    fig.suptitle(f"{name}: tracking (sim seed {seed})", fontsize=16)
    plt.show()

    fig, axs = plt.subplots(2, 2, figsize=(14, 9), sharex=True, layout="constrained")
    for j, s in enumerate(("S2", "S3", "S4")):
        axs[0, 0].plot(t, run["cmd"][:, j], **line(s, n))
        axs[0, 1].plot(t, log["p"][:, j], **line(s, n))
    for j, a in enumerate("xy"):
        axs[1, 0].plot(t, log["e"][:, j], **line(f"e {a}", n, label=f"prediction error e, {a}"))
        axs[1, 1].plot(t, log["w"][:, j], **line(f"w {a}", n, label=f"offset w, {a}"))
    axs[0, 0].set(title="commanded pressure u", ylabel="psi", ylim=(-0.3, 9.3))
    axs[0, 1].set(title="measured segment pressure p", ylabel="psi")
    axs[1, 0].set(title="disturbance seen by the model: e = y − ŷ(k | k−N)", ylabel="mm", xlabel="tracking time (s)")
    axs[1, 1].set(title="offset w the controller subtracts (low-pass of e)", ylabel="mm", xlabel="tracking time (s)")
    for a in axs.ravel():
        a.grid(alpha=0.3)
        a.legend(loc="upper right", ncol=3 if a in axs[0] else 1)
    fig.suptitle(f"{name}: inside the controller", fontsize=16)
    plt.show()

    E = log["E"][:, :2]
    Ed = np.gradient(E, DT, axis=0)
    fig, axs = plt.subplots(1, 2, figsize=(14, 6), layout="constrained")
    for j, s in enumerate((slice(None), slice(k0, None))):
        for i, a in enumerate("xy"):
            axs[j].plot(E[s, i], Ed[s, i], **line(a, len(E[s]), label=f"{a} axis"))
        lim, ylim = 1.15 * np.abs(E[s]).max(), 1.15 * np.abs(Ed[s]).max()
        axs[j].axhline(0, color="0.6", lw=0.8)
        axs[j].axvline(0, color="0.6", lw=0.8)
        axs[j].set(xlim=(-lim, lim), ylim=(-ylim, ylim), xlabel="E (mm)", ylabel="Ė (mm/s)",
                   title="whole run" if j == 0 else "after settling")
        axs[j].grid(alpha=0.3)
        axs[j].legend(loc="upper right")
    fig.suptitle(f"{name}: error-space phase portrait", fontsize=16)
    plt.show()'''

# ------------------------------------------------------------------------------------ model-specific

M2_MODEL_MD = r"""## 3 — The model: M2

Identified from simulator data (`scripts/task_space_model.py` in the project repo; parameters copied here). Per axis
$i\in\{x,y,z\}$, tip $y$ (mm) and measured S2–S4 segment pressure $p$ (psi), both relative to the settled pose:
$$\tau\,\dot p = -p + G\,u(t-T), \qquad \dot y_i = -a_i\,y_i + a_i\,(Hp)_i$$
- **Actuator:** dead time $T$ = 0.5 s (50 steps), lag $\tau \approx 0.6$ s, gain $G$ (command → measured pressure).
- **Mechanics:** each axis is a first-order lag (rate $a_i$) toward $Hp$; $H$ maps pressure to tip (mm/psi). S3 drives x;
  S2 and S4 drive y with opposite signs; all three lift z.

State $X = (y, p) \in \mathbb R^6$, both measured (tip and pouch sensors):
$$\dot X = \begin{bmatrix} -\Lambda & \Lambda H \\ 0 & -\tfrac1\tau I\end{bmatrix} X + \begin{bmatrix} 0 \\ \tfrac1\tau \operatorname{diag}(G)\end{bmatrix} u(t-T), \qquad y = [\,I\ \ 0\,]X, \qquad \Lambda = \operatorname{diag}(a)$$"""

M3_MODEL_MD = r"""## 3 — The model: M3

M2's actuator with **second-order** (lumped-mass) mechanics per axis, identified from simulator data
(`scripts/task_space_model.py` in the project repo; parameters copied here). Per axis $i\in\{x,y,z\}$, tip $y$ (mm) and
measured S2–S4 segment pressure $p$ (psi), both relative to the settled pose:
$$\tau\,\dot p = -p + G\,u(t-T), \qquad \ddot y_i + 2\zeta_i\omega_i\,\dot y_i + \omega_i^2\,y_i = \omega_i^2\,(Hp)_i$$
- **Actuator:** dead time $T$ = 0.5 s (50 steps), lag $\tau \approx 0.6$ s, gain $G$ (command → measured pressure).
- **Mechanics:** a damped second-order response ($\omega$, $\zeta$) toward $Hp$; $H$ maps pressure to tip (mm/psi).
  M2 is the first-order limit of this model. In the model comparison M3 had the smallest free-run error.

State $X = (y, \dot y, p) \in \mathbb R^9$. $y$ and $p$ are measured; $\dot y$ is the backward difference of the measured
tip (a signal the hardware also has):
$$\dot X = \begin{bmatrix} 0 & I & 0 \\ -\Omega^2 & -2Z\Omega & \Omega^2 H \\ 0 & 0 & -\tfrac1\tau I\end{bmatrix} X + \begin{bmatrix} 0 \\ 0 \\ \tfrac1\tau\operatorname{diag}(G)\end{bmatrix} u(t-T), \qquad y = [\,I\ \ 0\ \ 0\,]X$$"""


def model_code(kind: str, act: dict, m2: dict, m3: dict) -> str:
    vals = {"__GAIN__": repr(act["gain"]), "__TAU__": repr(act["tau_s"]), "__DELAY__": str(act["delay_steps"]),
            "__A__": repr(m2["a"]), "__H2__": repr(m2["H"]), "__WN__": repr(m3["wn"]), "__ZETA__": repr(m3["zeta"]),
            "__H3__": repr(m3["H"])}
    src = MODEL_HEAD + (MODEL_M2 if kind == "M2" else MODEL_M3)
    for k, v in vals.items():
        src = src.replace(k, v)
    return src


MODEL_HEAD = '''ACT = {"gain": np.array(__GAIN__),   # command -> measured pressure, S2-S4
       "tau_s": __TAU__,          # pneumatic lag, s
       "delay_steps": __DELAY__}                          # dead time 0.5 s at 100 Hz
'''

MODEL_M2 = '''M2 = {"a": np.array(__A__),   # 1/s, x y z
      "H": np.array(__H2__)}   # mm/psi, rows x y z, columns S2 S3 S4
MODEL_NAME, HORIZON, R_U = "M2", 100, 1e-5   # horizon 100 steps = 1 s after the dead time; R_U tuned in section 6


def model_matrices():
    """Continuous-time M2: state (y, p), output y."""
    a, H, O = M2["a"], M2["H"], np.zeros((3, 3))
    Ac = np.block([[-np.diag(a), np.diag(a) @ H], [O, -np.eye(3) / ACT["tau_s"]]])
    Bc = np.vstack([O, np.diag(ACT["gain"]) / ACT["tau_s"]])
    return Ac, Bc, np.hstack([np.eye(3), O])


def measure_state(y, y_prev, p):
    """Measured state (y, p): tip mm and S2-S4 pressure psi, both from the settled pose."""
    return np.concatenate([y, p])


md_table(["Axis", "a (1/s)", "H row: S2, S3, S4 (mm/psi)"],
         [[ax, f"{M2['a'][i]:.3f}", ", ".join(f"{h:+.3f}" for h in M2["H"][i])] for i, ax in enumerate("xyz")])
print(f"tau = {ACT['tau_s']:.3f} s, G = {np.round(ACT['gain'], 3)}, dead time = {ACT['delay_steps'] * DT} s")'''

MODEL_M3 = '''M3 = {"wn": np.array(__WN__),     # rad/s, x y z
      "zeta": np.array(__ZETA__),   # damping ratio
      "H": np.array(__H3__)}   # mm/psi, rows x y z, columns S2 S3 S4
MODEL_NAME, HORIZON, R_U = "M3", 100, 1e-5   # horizon 100 steps = 1 s after the dead time; R_U tuned in section 6


def model_matrices():
    """Continuous-time M3: state (y, y_dot, p), output y."""
    w, z, H, O, I = M3["wn"], M3["zeta"], M3["H"], np.zeros((3, 3)), np.eye(3)
    Ac = np.block([[O, I, O], [-np.diag(w ** 2), -np.diag(2 * z * w), np.diag(w ** 2) @ H], [O, O, -I / ACT["tau_s"]]])
    Bc = np.vstack([O, O, np.diag(ACT["gain"]) / ACT["tau_s"]])
    return Ac, Bc, np.hstack([I, O, O])


def measure_state(y, y_prev, p):
    """Measured state (y, y_dot, p); y_dot is the backward difference of the measured tip."""
    yd = np.zeros(3) if y_prev is None else (y - y_prev) / DT
    return np.concatenate([y, yd, p])


md_table(["Axis", "ω (rad/s)", "ζ", "H row: S2, S3, S4 (mm/psi)"],
         [[ax, f"{M3['wn'][i]:.2f}", f"{M3['zeta'][i]:.3f}", ", ".join(f"{h:+.3f}" for h in M3["H"][i])]
          for i, ax in enumerate("xyz")])
print(f"tau = {ACT['tau_s']:.3f} s, G = {np.round(ACT['gain'], 3)}, dead time = {ACT['delay_steps'] * DT} s")'''


def intro(kind: str) -> str:
    other = "M3" if kind == "M2" else "M2"
    blurb = ("M2, the first-order model identified from simulator data" if kind == "M2" else
             "M3, M2's actuator with second-order mechanics, identified from simulator data")
    return f"""# Soft Robotic Arm — Controller: Delay-Compensated MPC on {kind}

Submitted controller: a model-predictive controller built on **{blurb}**. It looks across the arm's 0.5 s dead time
with the commands already sent, reads the known trajectory ahead, and computes each command in closed form.

Standalone: runs on Google Colab (or any Python ≥ 3.10) with no project files. It installs the simulator package
(`soft-robotic-arm` 0.4.2), defines the model with its fitted parameters, the controller, the tests and the plots.

- **Plant:** `make_calibrated_sim(control_hz=100)`: three inputs (S2–S4, 0–9 psi), 0.5 s dead time, 0.6 s lag.
- **Interface:** `compute(t, obs, ref)` → absolute S2–S4 psi, from tip position and pouch pressures only.
- **Contents:** §3 model · §4 controller · §5 tests · §6 tuning · §7 official harness · §8 results · §9 plots · §10 summary.
- A companion notebook runs the same controller on {other}.

Run all cells (Runtime → Run all). About 2–3 minutes on Colab."""


def summary(kind: str) -> str:
    if kind == "M2":
        return """## 10 — Summary

Measured with this notebook (project repo, same code):

| Test | RMSE whole run (mm) | RMSE after settling (mm) |
|---|---:|---:|
| scored circle (official `evaluate_controller`) | 1.169 | 0.029 |
| held-out circle | 1.872 | 0.905 |
| triangle | 0.965 | 0.129 |
| square | 0.966 | 0.124 |
| squiggle | 0.260 | 0.071 |
| random waypoints | 0.173 | 0.167 |

- **Best on smooth circles:** 0.029 mm after the first lap on the scored circle, the lowest of the controllers we built.
- **Corners cost the most:** the error peaks at each vertex (triangle, square, waypoints). The MPC on M3, whose model
  adds the arm's second-order response, halves these errors, which suggests the first-order model is what limits
  this controller at corners.
- **Held-out circle is reach-limited:** about 8.2 mm of travel in x against a 10 mm target (only S3 drives x), and 23 %
  of commands saturate; every controller we tried is within 0.02 mm here.
- **Limitations:** simulation only; the tuned $R_u$ is the smallest tried, so commands are not smoothed; the 0.5 s dead
  time is assumed exact."""
    return """## 10 — Summary

Measured with this notebook (project repo, same code):

| Test | RMSE whole run (mm) | RMSE after settling (mm) |
|---|---:|---:|
| scored circle (official `evaluate_controller`) | 1.168 | 0.074 |
| held-out circle | 1.877 | 0.906 |
| triangle | 0.957 | 0.069 |
| square | 0.967 | 0.069 |
| squiggle | 0.245 | 0.041 |
| random waypoints | 0.124 | 0.105 |

- **Better on the polygons and the first random seed:** against the MPC on M2 (same horizon and tuning), M3 roughly
  halves the error on the triangle, square and squiggle (path seed 101) and cuts the waypoints by a third.
- **Not reliably better on random paths:** on the other seeds it can be much worse. Squiggle seed 103: 0.302 mm with
  66 % of commands saturated (M2: 0.029 mm). Waypoints seed 102: 0.281 mm, 51 % saturated (M2: 0.231 mm).
- **Worse on the smooth scored circle:** 0.074 mm after the first lap against 0.029 mm for M2.
- **Saturation is the problem:** 6–66 % of commands at 0 or 9 psi (0.4–23 % for M2). With $R_u = 10^{-5}$ the
  second-order model asks for sharper pressure changes than the valves allow, and the unconstrained optimum does not
  know about the 0–9 psi limits. A larger $R_u$, tuned on random paths, is the next step before using it.
- **Limitations:** simulation only; the 0.5 s dead time is assumed exact; $\\dot y$ is a raw backward difference, which
  real tip noise would amplify (the simulator's tip is noise-free)."""


def build(kind: str, act: dict, m2: dict, m3: dict) -> nbf.NotebookNode:
    cells = [md(intro(kind)), md("## 1 — Install"), code(INSTALL), md("## 2 — Imports, constants and plot style"),
             code(IMPORTS), md(M2_MODEL_MD if kind == "M2" else M3_MODEL_MD), code(model_code(kind, act, m2, m3)),
             md(MPC_MD), code(CONTROLLER.replace("MODEL_NAME", kind)), md(TESTS_MD), code(TESTS), md(TUNE_MD),
             code(TUNE), md(OFFICIAL_MD), code(OFFICIAL), md(RUN_MD), code(RUN), md(SEEDS_MD), code(SEEDS),
             md(PLOTS_MD), code(PLOTS)]
    for name in ("scored circle", "held-out circle", "triangle", "square", "squiggle", "random waypoints"):
        cells += [md(f"### {name}"), code(f'show("{name}")')]
    cells.append(md(summary(kind)))
    return nbf.v4.new_notebook(cells=cells, metadata={
        "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
        "colab": {"provenance": []}})


def main() -> None:
    act, m2, m3 = params()
    for kind in ("M2", "M3"):
        path = ROOT / "notebooks" / f"Soft_robotic_arm_MPC_{kind}.ipynb"
        nbf.write(build(kind, act, m2, m3), path)
        print(f"wrote {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
