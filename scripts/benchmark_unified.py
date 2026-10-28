"""One benchmark for every controller of both sessions: same tests, same loop, same metrics.

Controllers (gains exactly as in their source scripts; recorded per run in the JSON):
  scripts/compare_controllers.py   PD corrected, notebook SMC as written, notebook SMC
                                   corrected λ=8, paper SMC (M1), paper NDOSMC (M1),
                                   delay-aware SMC (M2, Taylor preview r + T r' + T² r''/2),
                                   delay-aware NDOSMC (M2)
  scripts/mpc_controller.py        its SMC baseline (λ=5, K=0.35, φ=0.02, deviation), the
                                   delay-compensated MPC with the predicted reference (default,
                                   RefPredictor: AR(2), then period replay) and the MPC with the
                                   true future reference (diagnostic, preview="oracle")
  scripts/benchmark_slide_tasks.py delay-aware SMC with exact r(t + T) (diagnostic, `Ahead`)
  new here                         delay-aware SMC fed r(t + T) predicted by mpc_controller's
                                   RefPredictor from the reference history only (no future
                                   knowledge): DelayAwareNDOSMC(exact_preview=True), same gains
                                   as the exact-preview one, wrapped by `Predicted`.
Its PD baseline (kp=500, kd=20, corrected allocator) is the PD corrected above, so it is not
repeated. Every controller is built by a factory make(ref_mm) that receives the task's
reference (mm, relative to home, with 4 s beyond the run), which only the two diagnostics use.

Tests (references imported from their source scripts, not re-derived):
  scored circle     CircularTrackingTask() (7 mm, 0.1 Hz), seed 0   compare_controllers.py
  held-out circle   10 mm, 0.15 Hz, seed 5                           compare_controllers.py
  motion tests      triangle, axial along y, axial along x, seed 0   motion_tests.py
  brief-scaled      axial, circle_ol, circle_static, circle_dyn,     benchmark_slide_tasks.py
                    triangle, seed 0
  recorded runs     axial, circular, triangular (lab robot), seed 0  benchmark_all_tasks.py
All on make_calibrated_sim(control_hz=100), run through benchmark_all_tasks.track (the loop of
evaluate_controller: 5 s settle at 4.5 psi, home = settled tip, commands clipped to 0-9 psi,
tip after each step scored against the reference passed to that step); 30 s of tracking
(180 s for the recorded runs, as in their script).

Metrics: benchmark_slide_tasks.score() for every run (planar RMSE whole run / after the first
lap, z RMSE, max planar error, y phase lag, saturation %, shape = RMS distance to the last lap's
path ignoring time, size % = path radius vs the reference's), lap = the task's period (10 s for
the recorded runs), plus mpc_controller.py's planar RMSE after the first 2 s.

The script first checks that it reproduces the source scripts' published numbers
(controller_comparison.json, mpc_controller.py's 1.220 mm, motion_tests.py's MPC numbers,
benchmark_slide_tasks.json, benchmark_all_tasks.json) and prints the differences.

Writes docs/figures/benchmark_unified.json and docs/figures/benchmark_unified.png.

Usage:
    uv run python scripts/benchmark_unified.py [--workers 10] [--upstream PATH]
"""

import argparse
import json
import multiprocessing as mp
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")       # one process per run; no BLAS oversubscription
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import benchmark_all_tasks as B  # noqa: E402
import benchmark_slide_tasks as BS  # noqa: E402
import compare_controllers as C  # noqa: E402
import motion_tests as MT  # noqa: E402
import mpc_controller as mpc  # noqa: E402
import plot_style as P  # noqa: E402
from soft_robotic_arm.evaluation import CircularTrackingTask, make_reference  # noqa: E402

FIG = B.FIG
HZ = B.HZ
PAD = 400                  # reference samples beyond the run, for the future-reference diagnostics
SYNTH_S = 30.0
RECORDED_S = 180.0
RECORDED_LAP_S = 10.0
N_C = dict((nm, mk) for nm, _, mk in C.controllers())

# ------------------------------------------------------------------------ controllers


class Predicted:
    """Passes the controller r(t + T) predicted from the reference history (no future knowledge).

    One mpc_controller.RefPredictor (mode "periodic": AR(2) with intercept until the reference
    is seen to repeat, then the detected period replayed) per axis, on the reference in metres
    relative to the first reference's home (the settled tip), as DelayMPCController uses it.
    T = the controller's dead-time steps (n_delay)."""

    def __init__(self, ctl, mode: str = "periodic"):
        self.c = ctl
        self.pred = [mpc.RefPredictor(mode) for _ in range(3)]
        self.home: np.ndarray | None = None

    def compute(self, t, obs, ref):
        if self.home is None:
            self.home = np.asarray(obs["tip_pos"], float).copy()
        r = np.asarray(ref, float) - self.home
        ahead = np.empty(3)
        for j, p in enumerate(self.pred):
            p.update(r[j])
            ahead[j] = p.predict(self.c.n_delay, 1)[0]
        return self.c.compute(t, obs, self.home + ahead)


def mpc_oracle(ref_mm: np.ndarray):
    """MPC with the true future reference: oracle(n) = home + reference at sample n."""
    ctl = mpc.DelayMPCController(preview="oracle")
    ctl.oracle = lambda n: ctl.tip0 + ref_mm[n, :2] / 1000.0   # tip0 = settled tip, set on the first call
    return ctl


def mpc_smc():
    S = C.S                                     # baselines() relies on FIX_ALLOCATOR at construction
    make = mpc.baselines()["SMC lam=5 K=0.35 phi=0.02, deviation"]
    assert S.FIX_ALLOCATOR
    return make()


def da_exact_preview():
    return C.N.DelayAwareNDOSMC(C.N.TaskSpaceModel.load(), lam=8.0, mu=1.0, eps=1.0, observer=False,
                                exact_preview=True)


# (label, source, family, make(ref_mm)); short labels and plot identities: plot_style.SERIES
CONTROLLERS = [
    ("PD, corrected allocator", "compare_controllers.py", "baseline",
     lambda ref: N_C["PD, corrected allocator"]()),
    ("SMC (notebook model), as written", "compare_controllers.py", "notebook",
     lambda ref: N_C["SMC (notebook model), as written"]()),
    ("SMC (notebook model), corrected allocator + deviation, λ=8",
     "compare_controllers.py", "notebook",
     lambda ref: N_C["SMC (notebook model), corrected allocator + deviation, λ=8"]()),
    ("SMC lam=5 K=0.35 phi=0.02, deviation (mpc_controller baseline)",
     "mpc_controller.py", "notebook", lambda ref: mpc_smc()),
    ("Paper SMC, eq. 5 (M1)", "compare_controllers.py", "paper",
     lambda ref: N_C["Paper SMC, eq. 5 (M1)"]()),
    ("Paper NDOSMC, eq. 12 (M1)", "compare_controllers.py", "paper",
     lambda ref: N_C["Paper NDOSMC, eq. 12 (M1)"]()),
    ("Delay-aware SMC, no observer (M2)", "compare_controllers.py", "delay",
     lambda ref: N_C["Delay-aware SMC, no observer (M2)"]()),
    ("Delay-aware NDOSMC (M2)", "compare_controllers.py", "delay",
     lambda ref: N_C["Delay-aware NDOSMC (M2)"]()),
    ("Delay-aware SMC, RefPredictor r(t+T) (M2)", "new (benchmark_unified.py)",
     "delay", lambda ref: Predicted(da_exact_preview())),
    ("Delay-aware SMC, exact future reference (M2) [diagnostic]",
     "benchmark_slide_tasks.py", "diagnostic", lambda ref: BS.Ahead(da_exact_preview(), ref)),
    ("MPC, predicted reference", "mpc_controller.py", "mpc",
     lambda ref: mpc.DelayMPCController()),
    ("MPC, true future reference [diagnostic]", "mpc_controller.py",
     "diagnostic", mpc_oracle),
]
CTRL = {c[0]: c for c in CONTROLLERS}


STATE = {"n", "t0", "u_prev", "z", "dt"}   # running state / derived, not settings


def gains(obj, depth: int = 0) -> dict:
    """Scalar settings of a constructed controller (and of what it wraps), for the record."""
    out = {}
    for k, v in vars(obj).items():
        if (isinstance(v, (bool, int, float, str, np.floating, np.integer)) and k not in STATE
                and not k.startswith("g_") and not (k == "k" and isinstance(obj, BS.Ahead))):
            out[k] = v.item() if isinstance(v, np.generic) else v
    if depth < 2:
        for k in ("c", "obs", "act"):
            if k in vars(obj) and hasattr(vars(obj)[k], "__dict__"):
                out[k] = gains(vars(obj)[k], depth + 1)
        if isinstance(obj, mpc.DelayMPCController):
            out.update(rho=mpc.RHO, horizon=obj.axes[0].horizon, delay_samples=obj.axes[0].d,
                       model=str(mpc.MODEL_PATH.relative_to(mpc.ROOT)))
            if obj.preview != "oracle":
                out["predictor_mode"] = obj.pred[0].mode
        if isinstance(obj, Predicted):
            out["predictor_mode"] = obj.pred[0].mode
    if hasattr(obj, "act") and hasattr(obj.act, "tau_s"):
        out["actuator"] = dict(tau_s=float(obj.act.tau_s), delay_s=float(obj.act.delay_s),
                               gain=np.asarray(obj.act.gain).tolist())
    return out


# ------------------------------------------------------------------------------ tests

def _fn_ref(fn, n: int) -> np.ndarray:
    """mm offsets from home of a reference function fn(home, k) (offset is home-independent)."""
    return np.array([1000.0 * fn(np.zeros(3), k) for k in range(n)])


def build_tests(upstream: Path) -> dict:
    """Every test: ref_fn(home, k) -> absolute m (the source's own definition), ref_mm (n + PAD, 3),
    n steps, seed, lap_s, group, and its parameters."""
    tests = {}
    n = int(round(SYNTH_S * HZ))

    def add(key, group, ref_fn, seed, lap_s, steps, source, **params):
        tests[key] = dict(group=group, ref_fn=ref_fn, ref_mm=_fn_ref(ref_fn, steps + PAD) if ref_fn else None,
                          seed=seed, lap_s=lap_s, n=steps, source=source, params=params)

    for (name, (task, seed)), key in zip(C.TASKS.items(), ("scored circle", "held-out circle")):
        add(key, "circles (compare_controllers)",
            lambda home, k, t=task: make_reference(home, k / HZ, t), seed, 1.0 / task.frequency_hz,
            task.track_steps, "compare_controllers.py TASKS", label=name, radius_m=task.radius_m,
            frequency_hz=task.frequency_hz)
    for motion in ("triangle", "axial along y", "axial along x"):
        fn = MT.MOTIONS[motion]
        add(f"MT {motion}", "motion_tests", lambda home, k, f=fn: f(home, k / MT.TASK.control_hz), 0,
            1.0 / MT.TASK.frequency_hz, MT.TASK.track_steps, "motion_tests.py MOTIONS",
            radius_m=MT.TASK.radius_m, frequency_hz=MT.TASK.frequency_hz)

    slide, radii = BS.build_tasks(n + PAD, BS.load_m2())
    for key, task in slide.items():
        ref = task["ref_mm"]
        tests[f"brief {key}"] = dict(
            group="brief-scaled (benchmark_slide_tasks)", ref_fn=lambda home, k, d=ref: home + d[k] / 1000.0,
            ref_mm=ref, seed=BS.SEED, lap_s=task["lap_s"], n=n, source="benchmark_slide_tasks.py build_tasks",
            params=dict(title=task["title"], **radii))

    nr = int(round(RECORDED_S * HZ))
    runs, _, psi = B.recorded_runs(upstream, RECORDED_S)
    Rz = B.yaw_matrix(psi)
    for w in B.RUNS:
        disp = runs[w]["disp"] @ Rz.T                     # n + 1 samples, as benchmark_all_tasks uses
        longer = B.load_run(upstream, B.RUNS[w], RECORDED_S + PAD / HZ)["disp"] @ Rz.T
        full = np.vstack([disp, longer[len(disp):nr + PAD]])  # preview beyond the run only
        tests[f"recorded {w}"] = dict(
            group="recorded robot runs (benchmark_all_tasks)", ref_fn=lambda home, k, d=full: home + d[k] / 1000.0,
            ref_mm=full, seed=B.SIM_SEED, lap_s=RECORDED_LAP_S, n=nr, source="benchmark_all_tasks.py recorded_runs",
            params=dict(run=B.RUNS[w], upstream_rev=B.UPSTREAM_REV, yaw_deg=float(np.degrees(psi))))
    return tests


# ---------------------------------------------------------------------------- running

TESTS: dict = {}          # filled in main() before the pool forks


def run_one(job: tuple[str, str]) -> dict:
    test_key, ctrl_name = job
    test = TESTS[test_key]
    _, source, family, make = CTRL[ctrl_name]
    short = P.label(ctrl_name)
    t0 = time.perf_counter()
    ctl = make(test["ref_mm"])
    g = gains(ctl)
    res = B.track(ctl, test["ref_fn"], test["n"], seed=test["seed"])
    tip = 1000.0 * (res["tips"] - res["home"])
    ref = 1000.0 * (res["refs"] - res["home"])
    m = BS.score(tip, ref, test["lap_s"], res["cmds"])
    lat = np.linalg.norm(tip[:, :2] - ref[:, :2], axis=1)
    m["planar_rmse_after_2s_mm"] = float(np.sqrt(np.mean(lat[int(2 * HZ):] ** 2)))
    return dict(test=test_key, group=test["group"], controller=ctrl_name, short=short, source=source,
                family=family, gains=g, seed=test["seed"], sim=f"make_calibrated_sim(control_hz={HZ}, "
                f"seed={test['seed']})", track_s=test["n"] / HZ, lap_s=test["lap_s"],
                test_params=test["params"], test_source=test["source"], metrics=m,
                run_s=time.perf_counter() - t0)


# ----------------------------------------------------------------------- sanity checks

def sanity_checks(rows: dict) -> list[tuple[str, float, float]]:
    """(label, published, reproduced) pairs from the source scripts' outputs."""
    out = []
    cc = json.loads((FIG / "controller_comparison.json").read_text())
    for name, key in zip(C.TASKS, ("scored circle", "held-out circle")):
        for ctrl, v in cc[name].items():
            if ctrl in CTRL:
                out.append((f"compare_controllers {key}: {ctrl}", v["rmse_mm"],
                            rows[(key, ctrl)]["metrics"]["planar_rmse_mm"]))
    out.append(("mpc_controller.py scored circle MPC (printed 1.220)", 1.220,
                rows[("scored circle", "MPC, predicted reference")]["metrics"]["planar_rmse_mm"]))
    for motion, v in (("triangle", 1.510), ("axial along y", 0.480), ("axial along x", 0.511)):
        out.append((f"motion_tests.py {motion} MPC (printed)", v,
                    rows[(f"MT {motion}", "MPC, predicted reference")]["metrics"]["planar_rmse_mm"]))
    bs = json.loads((FIG / "benchmark_slide_tasks.json").read_text())
    for task, info in bs["tasks"].items():
        for ctrl, v in info["controllers"].items():
            mine = {"Delay-aware SMC, exact future reference (M2)":
                    "Delay-aware SMC, exact future reference (M2) [diagnostic]"}.get(ctrl, ctrl)
            if mine in CTRL:
                for metric in ("planar_rmse_mm", "planar_rmse_after_lap_mm", "shape_rms_after_lap_mm",
                               "size_pct_after_lap", "saturated_pct"):
                    out.append((f"benchmark_slide_tasks {task}: {ctrl} {metric}", v[metric],
                                rows[(f"brief {task}", mine)]["metrics"][metric]))
    ba = json.loads((FIG / "benchmark_all_tasks.json").read_text())
    for w, info in ba["tasks"].items():
        for ctrl, v in info["controllers"].items():
            if ctrl in CTRL:
                out.append((f"benchmark_all_tasks {w}: {ctrl}", v["planar_rmse_mm"],
                            rows[(f"recorded {w}", ctrl)]["metrics"]["planar_rmse_mm"]))
    return out


# ----------------------------------------------------------------------------- figure

def figure(rows: dict, tests: list[str], ctrls: list[str], path: Path) -> None:
    """Heat maps controllers x tests (cividis, value in every cell), one metric per panel."""
    P.apply()
    get = lambda m: np.array([[rows[(t, c)]["metrics"][m] for t in tests] for c in ctrls])  # noqa: E731
    size = get("size_pct_after_lap")
    panels = [
        ("Planar RMSE after the first lap (mm; colour log scale)", get("planar_rmse_after_lap_mm"), "log", None),
        ("Shape error after the first lap (mm, timing ignored; colour log scale)",
         get("shape_rms_after_lap_mm"), "log", None),
        ("Path size, % of the reference (colour = |size − 100 %|, log scale)", np.abs(size - 100.0), "log", size),
        ("Commands at 0 or 9 psi (%)", get("saturated_pct"), "lin", None),
    ]
    fig, axs = plt.subplots(len(panels), 1, figsize=(P.WIDTH, 7.2 * len(panels)), layout="constrained")
    for ax, (title, v, kind, text) in zip(axs, panels):
        norm = (matplotlib.colors.LogNorm(vmin=max(0.02, float(np.nanmin(v[v > 0]))), vmax=float(np.nanmax(v)))
                if kind == "log" else matplotlib.colors.Normalize(0.0, 100.0))
        im = ax.imshow(np.clip(v, norm.vmin, norm.vmax), cmap="cividis", norm=norm, aspect="auto")
        shown = v if text is None else text
        fmt = (lambda x: f"{x:.0f}") if (text is not None or kind == "lin") else (
            lambda x: f"{x:.2f}" if x < 10 else f"{x:.1f}")
        P.annotate_cells(ax, np.clip(v, norm.vmin, norm.vmax), im, fmt, shown)
        ax.set_xticks(range(len(tests)), [t.replace(" along ", " ") for t in tests], rotation=40, ha="right")
        ax.set_yticks(range(len(ctrls)), [P.label(c) for c in ctrls])
        ax.set_title(title, loc="left")
        ax.set_xticks(np.arange(len(tests) + 1) - 0.5, minor=True)
        ax.set_yticks(np.arange(len(ctrls) + 1) - 0.5, minor=True)
        ax.grid(False)
        ax.grid(which="minor", color="white", lw=2)
        ax.tick_params(which="minor", length=0)
        fig.colorbar(im, ax=ax, fraction=0.03, pad=0.01)
    fig.suptitle("Unified benchmark (make_calibrated_sim, one harness loop; 30 s, recorded runs 180 s).\n"
                 "[diag] = sees the true future reference.")
    P.save(fig, path)


# ------------------------------------------------------------------------------- main

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--upstream", type=Path, default=B.UPSTREAM, help="clone of Jeevan-HM/Soft-Robotic-Arm")
    args = ap.parse_args()
    t_start = time.perf_counter()
    TESTS.update(build_tests(args.upstream))
    print(f"built {len(TESTS)} tests ({time.perf_counter() - t_start:.0f} s)", flush=True)

    tests, ctrls = list(TESTS), [c[0] for c in CONTROLLERS]
    jobs = [(t, c) for t in tests for c in ctrls]
    jobs.sort(key=lambda j: -TESTS[j[0]]["n"])                 # long recorded runs first
    rows = {}
    with mp.get_context("fork").Pool(args.workers) as pool:
        for i, r in enumerate(pool.imap_unordered(run_one, jobs), 1):
            rows[(r["test"], r["controller"])] = r
            if i % 12 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)} runs ({time.perf_counter() - t_start:.0f} s)", flush=True)

    checks = sanity_checks(rows)
    print("\nSanity checks: published vs reproduced (|diff|); printed values are 3 decimals")
    explained = lambda lab: "exact future reference" in lab and lab.endswith("saturated_pct")  # noqa: E731
    worst = 0.0
    for label, pub, mine in checks:
        tol = 0.0005 if "printed" in label else 1e-9
        ok = abs(pub - mine) <= tol
        if "printed" not in label and not explained(label):
            worst = max(worst, abs(pub - mine))
        note = "" if ok else ("   (expected: see below)" if explained(label) else "   <-- MISMATCH")
        if not ok or "printed" in label or "Delay-aware SMC, no" in label:
            print(f"  {label:<100} {pub:9.4f} {mine:9.4f} {abs(pub - mine):9.2e}{note}")
    print(f"  largest |diff| over the other {sum('printed' not in c[0] and not explained(c[0]) for c in checks)} "
          f"JSON comparisons: {worst:.2e}")
    print("  expected: benchmark_slide_tasks' Ahead clamps the reference at the run end; here it continues 4 s past\n"
          "  it, which changes only the commands of the last 0.5 s (saturation %); those never reach the tip\n"
          "  within the run (0.5 s dead time), so every tracking metric is identical.")

    for metric, title in (("planar_rmse_after_lap_mm", "planar RMSE after the first lap, mm"),
                          ("shape_rms_after_lap_mm", "shape error after the first lap, mm"),
                          ("size_pct_after_lap", "size %, after the first lap"),
                          ("saturated_pct", "% of commands at 0 or 9 psi"),
                          ("planar_rmse_mm", "planar RMSE whole run, mm"),
                          ("planar_rmse_after_2s_mm", "planar RMSE after the first 2 s, mm")):
        print(f"\n{title}")
        print(f"  {'':<38}" + "".join(f"{t.replace('recorded ', 'rec ').replace('brief ', 'b ')[:11]:>12}"
                                      for t in tests))
        for c in ctrls:
            fmt = "{:>12.0f}" if metric in ("size_pct_after_lap", "saturated_pct") else "{:>12.3f}"
            print(f"  {P.label(c):<38}" + "".join(fmt.format(rows[(t, c)]["metrics"][metric]) for t in tests))

    figure(rows, tests, ctrls, FIG / "benchmark_unified.png")
    total = time.perf_counter() - t_start
    out = dict(
        description=__doc__.split("\n\n")[0],
        harness="benchmark_all_tasks.track: make_calibrated_sim(control_hz=100, seed), settle 5 s at 4.5 psi, "
                "home = settled tip, commands clipped to 0-9 psi, tip after each step vs the reference passed",
        metrics="benchmark_slide_tasks.score (lap = the test's lap_s) + planar_rmse_after_2s_mm",
        workers=args.workers, total_run_s=total,
        controllers=[dict(name=c[0], short=P.label(c[0]), source=c[1], family=c[2]) for c in CONTROLLERS],
        tests={k: dict(group=v["group"], seed=v["seed"], lap_s=v["lap_s"], track_s=v["n"] / HZ,
                       source=v["source"], params=v["params"]) for k, v in TESTS.items()},
        sanity_checks=[dict(label=a, published=b, reproduced=c) for a, b, c in checks],
        runs=[rows[(t, c)] for t in tests for c in ctrls],
    )
    (FIG / "benchmark_unified.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
    print(f"\ntotal {total:.0f} s on {args.workers} workers; wrote {FIG / 'benchmark_unified.json'} and "
          f"{FIG / 'benchmark_unified.png'}")


if __name__ == "__main__":
    main()
