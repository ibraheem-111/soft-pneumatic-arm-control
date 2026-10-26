"""Benchmark every controller on the project brief's three test motions (slides 15-20).

The brief (RAS556 Course Project Intro, docs/assignment.md "Project brief: test motions")
shows axial, circular and triangular motions as open-loop demos from an older simulator
(tau ~120 ms, cm-scale tip motion). Shape and timing are kept here; amplitudes are scaled
to what make_calibrated_sim can reach (S1 is a sealed reservoir, S2-S4 commandable, 0-9 psi,
S2 -> +y, S3 -> -x, S4 -> -y). Tasks:

  axial          slide command S4 = 3.6 + 3.6 sin(2 pi 0.5 t) psi (0-7.2 psi), S2 and S3 held
                 at the 4.5 psi baseline; reference = the open-loop tip path (xyz) this
                 produces, relative to the settled pose. (S2/S3 at 1.5 psi, the bottom of
                 the identification range, is run open-loop for comparison only.)
  circle_ol      phase-shifted sines on S2-S4 at 0.4 Hz, 4.5 +- CIRCLE_OL_AMP psi, phases =
                 column azimuths (90, 180, 270 deg); reference = open-loop tip path (xyz).
                 With S1 sealed, x is driven by S3 alone, so this is a 2:1 ellipse.
  circle_static  geometric circle, 0.4 Hz, largest radius whose static command (M2 H times
                 actuator gain, minimum-norm over S2-S4) stays within 0-9 psi.
  circle_dyn     same circle with the largest radius M2 can follow at 0.4 Hz in steady
                 state (mechanics + 0.6 s actuator lag gain at 0.4 Hz).
  triangle       equilateral triangle, vertices at 0/120/240 deg, linear Cartesian
                 interpolation, 7.5 s per lap, centred on the settled pose; circumradius =
                 largest whose quasi-static command along the edges (position + constant edge
                 velocity through the M2 mechanics and actuator lags) stays within 0-9 psi.
Geometric references keep z at the settled value (as make_reference does).

Evaluation is benchmark_all_tasks.track (evaluate_controller's loop): make_calibrated_sim
(control_hz=100, seed=0), 5 s settle at 4.5 psi, home = settled tip, 30 s of tracking, the
tip after each step scored against the reference passed to that step. Metrics: planar (xy)
RMSE whole run and after the first lap, z RMSE (meaningful where z is part of the
reference), max planar error, % of commands at 0 or 9 psi, y-axis phase lag (package
cross-correlation). Controllers and gains: scripts/compare_controllers.py, not retuned.

Also reported: the open-loop replay of the generating commands (tasks axial, circle_ol;
exactly 0 because the simulator's tip is deterministic, the seed only drives pressure-sensor
noise), the score of a "perfect but delayed" tracker that does not predict the reference
(reference through the 0.5 s dead time, and through dead time plus the 0.6 s first-order
lag; controllers that extrapolate a periodic reference can beat it, so it is not a lower
bound), and numerical reach checks (static holds at 0/9 psi, full-amplitude 0.4 Hz sine on S3).

Writes docs/figures/benchmark_slide_tasks.json and docs/figures/slide_task_<task>.png,
docs/figures/slide_tasks_rmse.png.

Usage:
    uv run python scripts/benchmark_slide_tasks.py [--track-s 30]
"""

import argparse
import json
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import benchmark_all_tasks as B
import compare_controllers as C
from soft_robotic_arm import make_calibrated_sim
from soft_robotic_arm.evaluation import CircularTrackingTask, _phase_lag_seconds, make_reference

FIG = B.FIG
HZ, SEED, BASE, LIMIT = B.HZ, B.SIM_SEED, B.BASELINE_PSI, B.LIMIT_PSI
SPAN = min(BASE, LIMIT - BASE)                 # +-4.5 psi around the baseline
ID_RANGE_PSI = (1.5, 7.5)                      # broadband identification levels, task_space_model.py
AXIAL_F, AXIAL_MEAN, AXIAL_AMP = 0.5, 3.6, 3.6 # slide 15-16: Seg 4, 0.5 Hz, 0-7.2 psi
CIRCLE_F = 0.4                                 # slide 17-18
CIRCLE_OL_AMP = 3.0                            # psi: 1.5-7.5, the identified range
TRI_PERIOD = 7.5                               # slide 19-20, s per lap
AZIMUTH = np.radians([90.0, 180.0, 270.0])     # S2, S3, S4

SHOW = {  # controllers drawn in the per-task figures, with colours
    "Delay-aware SMC, no observer (M2)": "#6a4c9c",
    "Paper SMC, eq. 5 (M1)": "#9a6a12",
    "PD, corrected allocator": "#8a8f93",
    "SMC (notebook model), corrected allocator + deviation, λ=8": "#1f6f7a",
}
BEST = "Delay-aware SMC, no observer (M2)"
SHORT = {
    "PD, notebook allocator": "PD (as written)",
    "PD, corrected allocator": "PD corrected",
    "SMC (notebook model), as written": "Notebook SMC (as written)",
    "SMC (notebook model), corrected allocator + deviation, λ=8": "Notebook SMC λ=8",
    "Paper SMC, eq. 5 (M1)": "Paper SMC",
    "Paper NDOSMC, eq. 12 (M1)": "Paper NDOSMC",
    "Delay-aware SMC, no observer (M2)": "Delay-aware SMC",
    "Delay-aware NDOSMC (M2)": "Delay-aware NDOSMC",
}


# ----------------------------------------------------------------- model-based reach

def load_m2() -> dict:
    """M2 (docs/figures/task_space_model.json): H (measured psi -> mm), rates a, actuator gain, tau."""
    d = json.loads((FIG / "task_space_model.json").read_text())
    m = d["M2_mechanics"]
    H = np.array([m[ax]["H"] for ax in "xyz"])
    a = np.array([m[ax]["a"] for ax in "xyz"])
    g = np.array(d["actuator"]["gain"])
    return dict(H=H, a=a, g=g, Hc=H * g, tau=float(np.mean(d["actuator"]["tau"])),
                dead_s=float(np.mean(d["actuator"]["dead_s"])))


def command_per_mm(m: dict, X: np.ndarray, V: np.ndarray | None = None, omega: float = 0.0) -> np.ndarray:
    """Command deviation (psi, S2-S4) that holds planar X (N, 2) in M2's steady state.

    Minimum-norm over S2-S4 (pinv of H's x-y rows). With V (N, 2, edge velocity in mm/s) the
    quasi-static ramp demand H p = X + V/a, g u = p + tau dp/dt is used; with omega (rad/s)
    X is a complex phasor and the result is the amplitude of the sinusoidal command.
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
    for _ in range(int(round(B.SETTLE_S * HZ))):
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


def score(tip_mm: np.ndarray, ref_mm: np.ndarray, lap_s: float, cmds: np.ndarray | None = None) -> dict:
    err = tip_mm - ref_mm
    base = B.metrics(err, cmds, after_s=lap_s)
    late = np.arange(len(err)) / HZ >= lap_s
    out = dict(planar_rmse_mm=base["planar_rmse_mm"], planar_rmse_after_lap_mm=base["planar_rmse_after_5s_mm"],
               z_rmse_mm=base["z_rmse_mm"], z_rmse_after_lap_mm=base["z_rmse_after_5s_mm"],
               max_planar_error_mm=float(np.linalg.norm(err[:, :2], axis=1).max()),
               max_planar_error_after_lap_mm=float(np.linalg.norm(err[late, :2], axis=1).max()),
               phase_lag_s=_phase_lag_seconds(ref_mm[:, 1], tip_mm[:, 1], HZ, max_lag_s=0.5 * lap_s))
    if cmds is not None:
        out["saturated_pct"] = base["saturated_pct"]
    return out


# ------------------------------------------------------------------------ figures

def task_figure(key: str, task: dict, runs: dict, path) -> None:
    ref = task["ref_mm"]
    t = np.arange(len(ref)) / HZ
    last = t >= t[-1] - task["lap_s"]
    axial = key == "axial"
    fig = plt.figure(figsize=(15.5 if axial else 13.5, 4.4))
    gs = fig.add_gridspec(1, 4 if axial else 3, width_ratios=[1, 1, 1.3, 1.5] if axial else [1, 1.3, 1.5])
    planes = [(0, 1, "x"), (1, 2, "y")] if axial else [(0, 1, "x")]
    for j, (i0, i1, _) in enumerate(planes):
        ax = fig.add_subplot(gs[0, j])
        ax.plot(ref[last, i0], ref[last, i1], "--", color="black", lw=1.2, label="reference")
        for name, col in SHOW.items():
            tip = runs[name]["tip_mm"]
            ax.plot(tip[last, i0], tip[last, i1], color=col, lw=1.2, label=SHORT[name])
        lab = "xyz"
        ax.set(xlabel=f"{lab[i0]} − home (mm)", ylabel=f"{lab[i1]} − home (mm)",
               title=f"Tip path, last lap ({lab[i0]}-{lab[i1]})")
        if i1 != 2:
            ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
        if j == 0:
            ax.legend(fontsize=7.5, loc="best")
    ax = fig.add_subplot(gs[0, len(planes)])
    cm = runs[BEST]["cmds"]
    for s, col in zip(range(3), ["#c0392b", "#2e86c1", "#27ae60"]):
        ax.plot(t, cm[:, s], color=col, lw=0.9, label=f"S{s + 2}")
    ax.set(xlabel="tracking time (s)", ylabel="command (psi)", ylim=(-0.3, 9.3),
           title=f"{SHORT[BEST]}: commands")
    ax.legend(fontsize=8, ncol=3, loc="upper right")
    ax.grid(alpha=0.3)
    ax = fig.add_subplot(gs[0, len(planes) + 1])
    for name, col in SHOW.items():
        e = np.linalg.norm(runs[name]["tip_mm"][:, :2] - ref[:, :2], axis=1)
        ax.plot(t, e, color=col, lw=1.0, label=f"{SHORT[name]} ({task['controllers'][name]['planar_rmse_mm']:.2f})")
    fl = delayed_floor(ref, task["dead_s"], task["tau"], lag=True)
    ax.plot(t, np.linalg.norm(fl[:, :2] - ref[:, :2], axis=1), ":", color="black", lw=1.0,
            label=f"ref via 0.5 s delay + 0.6 s lag ({task['floor_delay_lag']['planar_rmse_mm']:.2f})")
    ax.set(xlabel="tracking time (s)", ylabel="planar error (mm)", title="Tracking error (xy RMSE mm)")
    ax.set_ylim(0, None)
    ax.legend(fontsize=7.5, loc="upper right")
    ax.grid(alpha=0.3)
    fig.suptitle(task["title"], fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def summary_figure(out: dict, path) -> None:
    keys = list(out["tasks"])
    names = [nm for nm, _, _ in C.controllers()]
    cols = ["#c9cccf", "#8a8f93", "#9fd3d9", "#1f6f7a", "#9a6a12", "#e0b45c", "#6a4c9c", "#b39ddb"]
    fig, ax = plt.subplots(figsize=(13, 4.6))
    w = 0.8 / len(names)
    x = np.arange(len(keys))
    for j, (nm, c) in enumerate(zip(names, cols)):
        v = [out["tasks"][k]["controllers"][nm]["planar_rmse_mm"] for k in keys]
        ax.bar(x - 0.4 + (j + 0.5) * w, v, w, color=c, label=SHORT[nm])
    for i, k in enumerate(keys):
        f = out["tasks"][k]["floor_delay_lag"]["planar_rmse_mm"]
        ax.plot([i - 0.42, i + 0.42], [f, f], color="black", lw=1.2, ls=":",
                label="reference via 0.5 s delay + 0.6 s lag (no prediction)" if i == 0 else None)
    ax.set_xticks(x, [out["tasks"][k]["short"] for k in keys], fontsize=8.5)
    ax.set(ylabel="planar RMSE, whole run (mm)", title="Brief test motions: xy RMSE per controller "
           f"(make_calibrated_sim seed {SEED}, {out['track_s']:g} s)")
    ax.set_ylim(0, 1.4 * ax.get_ylim()[1])
    ax.legend(fontsize=7.5, ncol=3, loc="upper left")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


# --------------------------------------------------------------------------- main

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--track-s", type=float, default=30.0)
    args = ap.parse_args()
    n = int(round(args.track_s * HZ))
    t_start = time.perf_counter()
    m = load_m2()
    bounds = reach_bounds(m)
    checks = reach_checks(m)
    r_static, r_dyn = floor01(bounds["circle_static_mm"]), floor01(bounds["circle_dyn_0p4hz_mm"])
    r_tri = floor01(bounds["triangle_edge_mm"])

    # references (mm, relative to home), with their generating commands where open loop
    ax_disp, ax_cmd = open_loop(axial_cmd(BASE), n)
    ax_low, _ = open_loop(axial_cmd(ID_RANGE_PSI[0]), n,
                          settle=[ID_RANGE_PSI[0], ID_RANGE_PSI[0], AXIAL_MEAN])
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

    out = dict(sim=f"make_calibrated_sim(control_hz={HZ}, seed={SEED})", track_s=args.track_s,
               settle=f"{B.SETTLE_S} s at {BASE} psi", m2=dict(tau_s=m["tau"], dead_s=m["dead_s"]),
               reach_bounds_m2=bounds, reach_checks=checks,
               chosen=dict(axial_hold_psi=BASE, axial_S4_psi=[AXIAL_MEAN - AXIAL_AMP, AXIAL_MEAN + AXIAL_AMP],
                           circle_ol_amp_psi=CIRCLE_OL_AMP, circle_static_r_mm=r_static,
                           circle_dyn_r_mm=r_dyn, triangle_R_mm=r_tri),
               axial_low_hold_open_loop=dict(
                   hold_psi=ID_RANGE_PSI[0], settle=f"S2/S3 {ID_RANGE_PSI[0]}, S4 {AXIAL_MEAN} psi",
                   range_mm=dict(min=ax_low.min(0).tolist(), max=ax_low.max(0).tolist())),
               tasks={})
    runs_all = {}
    for key, task in tasks.items():
        ref = task["ref_mm"]
        task.update(dead_s=m["dead_s"], tau=m["tau"])
        info = dict(title=task["title"], short=task["short"], lap_s=task["lap_s"],
                    z_in_reference=key in ("axial", "circle_ol"),
                    reference_range_mm=dict(min=ref.min(0).tolist(), max=ref.max(0).tolist()),
                    floor_delay=score(delayed_floor(ref, m["dead_s"], m["tau"], False), ref, task["lap_s"]),
                    floor_delay_lag=score(delayed_floor(ref, m["dead_s"], m["tau"], True), ref, task["lap_s"]),
                    controllers={})
        task["controllers"], task["floor_delay_lag"] = info["controllers"], info["floor_delay_lag"]
        ref_fn = lambda home, k, d=ref: home + d[k] / 1000.0  # noqa: E731
        if "cmd" in task:
            class OpenLoop:
                def __init__(self, c):
                    self.c, self.k = c, 0

                def compute(self, t, obs, r):
                    self.k += 1
                    return self.c[self.k - 1]
            res = B.track(OpenLoop(task["cmd"]), ref_fn, n)
            info["open_loop"] = score(1000.0 * (res["tips"] - res["home"]), ref, task["lap_s"], res["cmds"])
        runs = {}
        for name, _, make in C.controllers():
            t0 = time.perf_counter()
            res = B.track(make(), ref_fn, n)
            tip = 1000.0 * (res["tips"] - res["home"])
            row = score(tip, 1000.0 * (res["refs"] - res["home"]), task["lap_s"], res["cmds"])
            row["run_s"] = time.perf_counter() - t0
            info["controllers"][name] = row
            runs[name] = dict(tip_mm=tip, cmds=res["cmds"])
        runs_all[key] = runs
        out["tasks"][key] = info
        task_figure(key, task, runs, FIG / f"slide_task_{key}.png")
        print(f"  {key}: done ({time.perf_counter() - t_start:.0f} s elapsed)")
    summary_figure(out, FIG / "slide_tasks_rmse.png")

    print(f"\nM2 reach (|Hc| x 4.5 psi, min-norm): circle static {bounds['circle_static_mm']:.2f} mm, "
          f"circle at {CIRCLE_F} Hz {bounds['circle_dyn_0p4hz_mm']:.2f} mm, triangle static "
          f"{bounds['triangle_static_mm']:.2f} mm, triangle edges {bounds['triangle_edge_mm']:.2f} mm")
    for k, v in checks.items():
        print(f"  reach check {k}: measured {np.round(v['measured_mm'], 2)} predicted {np.round(v['predicted_mm'], 2)}")
    lo = out["axial_low_hold_open_loop"]["range_mm"]
    print(f"  axial with S2/S3 at {ID_RANGE_PSI[0]} psi (open loop, own settled pose): x "
          f"{lo['min'][0]:.2f}..{lo['max'][0]:.2f}, y {lo['min'][1]:.2f}..{lo['max'][1]:.2f}, "
          f"z {lo['min'][2]:.2f}..{lo['max'][2]:.2f} mm")
    print(f"\n{out['sim']}, {args.track_s:g} s. xy / z RMSE mm whole run and after the first lap; "
          "max>lap = max planar error after the first lap; sat = % commands at 0 or 9 psi; lag = y phase lag s")
    hdr = f"  {'':<28}{'xy':>7}{'xy>lap':>8}{'z':>7}{'z>lap':>7}{'max>lap':>8}{'sat%':>7}{'lag':>7}"
    for key, info in out["tasks"].items():
        rg = info["reference_range_mm"]
        print(f"\n{key.upper()}: {info['title']}\n  reference x {rg['min'][0]:.2f}..{rg['max'][0]:.2f}, "
              f"y {rg['min'][1]:.2f}..{rg['max'][1]:.2f}, z {rg['min'][2]:.2f}..{rg['max'][2]:.2f} mm")
        print(hdr)
        rows = [("open loop (generating cmds)", info.get("open_loop")),
                ("delayed, no prediction", info["floor_delay"]), ("delayed + lagged, no pred.", info["floor_delay_lag"])]
        rows += [(SHORT[nm], info["controllers"][nm]) for nm, _, _ in C.controllers()]
        for label, r in rows:
            if r is None:
                continue
            sat = f"{r['saturated_pct']:>7.1f}" if "saturated_pct" in r else f"{'':>7}"
            print(f"  {label:<28}{r['planar_rmse_mm']:>7.2f}{r['planar_rmse_after_lap_mm']:>8.2f}"
                  f"{r['z_rmse_mm']:>7.2f}{r['z_rmse_after_lap_mm']:>7.2f}{r['max_planar_error_after_lap_mm']:>8.2f}"
                  f"{sat}{r['phase_lag_s']:>7.2f}")
    out["total_run_s"] = time.perf_counter() - t_start
    (FIG / "benchmark_slide_tasks.json").write_text(json.dumps(out, indent=2))
    print(f"\ntotal {out['total_run_s']:.0f} s; wrote {FIG / 'benchmark_slide_tasks.json'}, "
          f"slide_task_<task>.png, slide_tasks_rmse.png")


if __name__ == "__main__":
    main()
