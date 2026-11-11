"""Build docs/reports/m2-controller-comparison.html from the results of compare_m2_controllers.py.

Reads docs/figures/m2_controllers.json and docs/figures/m2ctrl_<test>.png (figures embedded) and
writes one self-contained page: findings, setup, one table and figure per test, the SMC preview
comparison, the MPC horizon sensitivity and the R_u tuning. Best value per column (among the
controllers given the known reference) is bold with a star.

Usage:
    uv run python scripts/compare_m2_controllers.py      # results first
    uv run python scripts/report_m2_controllers.py
"""

import json
from html import escape

from softarm.constants import FIG, ROOT
from softarm.reports import ABOUT, CSS, head
from softarm.labels import MPC_F, NDOSMC_KNOWN, SMC_KNOWN, SMC_L4, mpc_label
from softarm.reports import img as _img

OUT = ROOT / "docs" / "reports" / "m2-controller-comparison.html"

SMC, NDO = SMC_KNOWN, NDOSMC_KNOWN
M2 = [mpc_label("M2", h) for h in (25, 100, 300)]
ROWS = [SMC, SMC_L4, NDO, *M2, MPC_F]
ORDER = ["scored circle", "held-out circle", "triangle", "square", "squiggle", "random waypoints"]


def img(test: str) -> str:
    return _img(f"m2ctrl_{test.replace(' ', '_')}.png", f"{test}: tip paths and planar error over time.")


def table(head: list, rows: list, best_cols: set, skip_best: set) -> str:
    """rows: (label, [(text, value or None)]); lowest value per column in best_cols gets the star."""
    best = {}
    for c in best_cols:
        vals = [r[1][c][1] for r in rows if r[0] not in skip_best and r[1][c][1] is not None]
        best[c] = min(vals) if vals else None
    h = "".join(f'<th{"" if i == 0 else " class=n"}>{escape(x)}</th>' for i, x in enumerate(head))
    body = ""
    for lab, cells in rows:
        tds = f"<td>{escape(lab)}</td>"
        for c, (txt, v) in enumerate(cells):
            star = c in best and v is not None and lab not in skip_best and v == best[c]
            tds += f'<td class="n{" best" if star else ""}">{escape(txt)}{" ★" if star else ""}</td>'
        body += f"<tr>{tds}</tr>"
    return f'<div class="tbl"><table><thead><tr>{h}</tr></thead><tbody>{body}</tbody></table></div>'


def test_section(test: str, R: dict, kind: str) -> str:
    after = "first 5 s" if kind == "open" else "first lap"
    head = ["Controller", "RMSE, whole run (mm)", f"RMSE after {after} (mm)", "Shape error (mm)",
            f"Max error after {after} (mm)"]
    m0 = R[(test, SMC)]
    if kind == "circle":
        head.append(f"Radius range (mm), ref {m0['radius_ref_mm']:.1f}")
    if kind == "polygon":
        head.append("Path size (%)")
    head += ["Commands at 0/9 psi (%)", "Lag (s)"]
    rows = []
    for n in ROWS:
        m = R[(test, n)]
        r = lambda k, f: (format(m[k], f), round(m[k], int(f[1])))  # noqa: E731
        cells = [r("rmse_whole_mm", ".2f"), r("rmse_settled_mm", ".3f"), r("shape_mm", ".3f"), r("max_settled_mm", ".2f")]
        if kind == "circle":
            cells.append((f"{m['radius_min_mm']:.1f}–{m['radius_max_mm']:.1f}", None))
        if kind == "polygon":
            cells.append((f"{m['size_pct']:.0f}", None))
        cells += [(f"{m['saturated_pct']:.1f}", None), (f"{m['lag_s']:+.2f}", None)]
        rows.append((n, cells))
    last = "last lap" if kind != "open" else "after 5 s"
    return (f'<section><h2>{escape(test)}</h2><p class="meta">{escape(ABOUT[test])} 30 s tracked after a 5 s settle.</p>'
            + table(head, rows, {0, 1, 2, 3}, set())
            + f'<figure><div class="scroll">{img(test)}</div><figcaption>Top: SMC, NDOSMC, the tuned MPC on M2 (all three '
            f'horizons give the same path, so one is drawn) and the MPC on F. Bottom: the M2 MPC horizons at R_u = 10, where '
            f'they differ. Left: tip path ({last}); right: '
            f'planar error over the whole run. Thick black dashed: reference.</figcaption></figure></section>')


def main() -> None:
    d = json.loads((FIG / "m2_controllers.json").read_text())
    R = {(r["test"], r["controller"]): r["metrics"] for r in d["runs"]}
    kind = {t: ("circle" if "radius_ref_mm" in R[(t, SMC)] else "polygon" if "size_pct" in R[(t, SMC)] else "open")
            for t in ORDER}
    v = lambda t, n: R[(t, n)]["rmse_settled_mm"]  # noqa: E731
    sz = lambda t, n: R[(t, n)]["size_pct"]  # noqa: E731
    findings = f"""<section><h2>Findings</h2><ul>
<li><b>With the reference known, the MPC on F is best on every path with corners or random turns</b> (RMSE after settling):
triangle {v('triangle', MPC_F):.3f} mm vs {v('triangle', M2[1]):.3f} (MPC on M2) and {v('triangle', SMC):.3f} (SMC);
square {v('square', MPC_F):.3f} vs {v('square', M2[1]):.3f} and {v('square', SMC):.3f}; waypoints
{v('random waypoints', MPC_F):.3f} vs {v('random waypoints', M2[1]):.3f} and {v('random waypoints', SMC):.3f}; squiggle
{v('squiggle', MPC_F):.3f} vs {v('squiggle', M2[1]):.3f} and {v('squiggle', SMC):.3f}.</li>
<li><b>On the circles the MPC on M2 is best:</b> scored circle {v('scored circle', M2[1]):.3f} mm vs {v('scored circle', MPC_F):.3f}
(F) and {v('scored circle', SMC):.3f} (SMC). The held-out circle is a tie ({min(v('held-out circle', n) for n in ROWS):.2f}–{max(v('held-out circle', n) for n in ROWS):.2f} mm):
it is limited by reach, about 8.2 mm in x against 10 mm for every controller, with 19–23 % of commands saturated.</li>
<li><b>At equal gain the disturbance observer lowers whole-run RMSE on every test, but not always the settled error:</b>
SMC at λ = 4 without vs with the observer, whole-run RMSE
""" + ", ".join(f"{t} {R[(t, SMC_L4)]['rmse_whole_mm']:.2f} → {R[(t, NDO)]['rmse_whole_mm']:.2f}" for t in ORDER) + """ mm
(after settling: """ + ", ".join(f"{t} {v(t, SMC_L4):.3f} → {v(t, NDO):.3f}" for t in ORDER) + """); it mostly removes the start-up
and corner errors, and is slightly worse once settled on the scored circle and the squiggle. The λ = 8 SMC without the
observer is close to the NDOSMC, so a higher gain recovers most of what the observer adds.</li>
<li><b>MPC horizon on M2:</b> with the tuned weight (R<sub>u</sub> = {d['r_u']['100']:g}) 25, 100 and 300 steps give identical results.
When command changes are penalised (R<sub>u</sub> = 10) 25 steps (0.25 s) is worse and 100 steps (1 s) equals 300 steps (3 s).</li>
<li><b>Whole-run RMSE is dominated by the start:</b> each reference starts away from, or accelerating from, the settled tip,
and nothing reaches the arm for 0.5 s.</li></ul></section>"""
    setup = r"""<section><h2>Setup</h2>
<p>Plant <code>make_calibrated_sim(control_hz=100)</code>; controllers run every 10 ms; 5 s settle at 4.5 psi, then 30 s
of tracking scored like the course harness. Metrics are planar (x–y) and in mm:</p>
<ul><li><b>RMSE, whole run</b>: root mean square of the tip-to-reference distance over the 30 s.</li>
<li><b>RMSE after settling</b>: the same after the first lap (circles, triangle, square) or the first 5 s (random paths).</li>
<li><b>Shape error</b>: RMS distance from the tip to the nearest point of the reference path, ignoring timing.</li>
<li><b>Radius range</b> (circles): smallest and largest distance of the traced path from its centre.</li>
<li><b>Path size</b> (triangle, square): RMS distance from the centre, traced over reference; 100 % is the right size.</li></ul>
<h3>Controllers</h3>
<ul><li><b>Delay-aware SMC</b>: M2 + Smith predictor + lag inverse, λ = 8 s⁻¹, η = 1 mm/s, μ = 1 mm, no observer; reads
r(t + 0.5 s) from the trajectory.</li>
<li><b>Delay-aware NDOSMC</b>: the same with the disturbance observer and Nikoletta's notebook gains (λ = 4, l₁ = 0.1,
μ = 1 mm, Φ = 1, ε = 1).</li>
<li><b>Delay-aware SMC, λ = 4</b>: the SMC at the NDOSMC's gain without the observer, so comparing it with the NDOSMC
isolates the observer (the λ = 8 SMC differs from the NDOSMC in both gain and observer).</li>
<li><b>MPC (M2)</b>: state \(X = (y, p)\) (tip and measured S2–S4 pressure, deviations from the settled pose), M2
discretised at 10 ms. Each step: predict 50 steps across the dead time with the commands already sent; add a model-error
correction \(w\) (measured tip minus the tip predicted 0.5 s earlier, low-pass 0.5 s); minimise over the next H commands
<div class="eqn">\[ J = \sum_{i=0}^{H-1} \big\| \hat y_{k+N+1+i} + w - r_{k+N+i} \big\|^2 + R_u \sum_{i=0}^{H-1} \big\| u_{k+i} - u_{k+i-1} \big\|^2,\qquad N = 50 \]</div>
in closed form (unconstrained), apply the first move, clip to 0–9 psi. <b>Horizon</b>: H steps of 10 ms after the 0.5 s
dead time, so 25 steps = 0.25 s (looks 0.5–0.75 s ahead), 100 steps = 1 s (0.5–1.5 s ahead), 300 steps = 3 s
(0.5–3.5 s ahead). R<sub>u</sub> (mm²/psi²) tuned per horizon on a separate circle (5 mm, 0.07 Hz, seed 5).</li>
<li><b>MPC (F)</b>: <code>softarm.controllers.KalmanDelayMPC</code> on the frequency-identified model F: one model per
planar channel (ν<sub>x</sub> = −u₃, ν<sub>y</sub> = u₂ − u₄), Kalman filter on tip and channel pressure, same delay
predictor, same cost with ρ = 10⁻⁵ m²/psi² (its own tuning), horizon 300 steps = 3 s; given the true future reference.</li></ul>
<p class="meta">Run interactively: <code>notebooks/m2_controller_comparison.ipynb</code>. Regenerate:
<code>uv run python scripts/compare_m2_controllers.py</code> (about 45 s), then <code>uv run python scripts/report_m2_controllers.py</code>.
Every run's gains, metrics and traces: <code>docs/figures/m2_controllers.json</code>.</p></section>"""
    hz = []
    for ru, lab in ((None, "tuned R_u = {:g}"), (1.0, "R_u = 1"), (10.0, "R_u = 10")):
        for h, n in zip((25, 100, 300), M2):
            key = n if ru is None else f"{n}, R_u = {ru:g}"
            hz.append((lab.format(d["r_u"][str(h)]), [(f"{h} steps = {h / 100:.2f} s", None)]
                       + [(f"{v(t, key):.3f}", None) for t in ORDER]))
    horizon = ('<section><h2>MPC (M2) horizon</h2><h3>RMSE after settling (mm) by horizon and command-change weight</h3>'
               + table(["Weight on command changes", "Horizon", *ORDER], hz, set(), set()))
    grid = sorted({x["r_u"] for x in d["r_u_tuning"]})
    tu = [(f"{h} steps = {h / 100:.2f} s", [(f"{x['rmse_after_lap_mm']:.4f}", None) for x in d["r_u_tuning"]
                                            if x["horizon_steps"] == h]) for h in (25, 100, 300)]
    horizon += ('<h3>Tuning (5 mm, 0.07 Hz circle, seed 5; RMSE after the first lap, mm)</h3>'
                + table(["Horizon", *[f"R_u = {g:g}" for g in grid]], tu, set(), set())
                + '<p class="meta">The tuning optimum is at the smallest weight tried, but the whole range 10⁻⁵–10⁻¹ is within '
                  '0.002 mm; a larger weight costs accuracy on smooth circles and gains it on cornered paths.</p></section>')
    intro = """<div class="wrap">
<header><div class="eyebrow">Soft pneumatic arm · controllers with the reference known in advance</div>
<h1>M2 Controller Comparison</h1>
<p class="meta">Delay-aware SMC, delay-aware NDOSMC and an MPC built on the identified model M2 (three horizons), plus the
MPC on the frequency-identified model F, each given the true future reference (confirmed acceptable by the professor), on
two circles, a triangle, a square and two random paths.</p></header>
"""
    html = head("M2 Controller Comparison") + CSS + "\n" + intro + findings + setup \
        + "".join(test_section(t, R, kind[t]) for t in ORDER) + horizon + "\n</div>\n"
    OUT.write_text(html)
    print(f"wrote {OUT.relative_to(ROOT)} ({len(html) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
