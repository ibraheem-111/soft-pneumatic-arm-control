"""Build docs/reports/smc-on-m2.html: the delay-aware SMC and NDOSMC on M2, derived and plotted.

Equations follow softarm.controllers.DelayAwareNDOSMC (the code, not the paper). Numbers and
figures come from scripts/smc_signals.py (docs/figures/smc_signals.json, smc_signals_<test>.png,
smc_phase_<test>.png). Same page style as report_m2_controllers.py (softarm.reports).

Usage:
    uv run python scripts/smc_signals.py          # results first
    uv run python scripts/report_smc_explained.py
"""

import json
from html import escape

import numpy as np

from softarm.constants import FIG, ROOT
from softarm.reports import ABOUT, CSS, head, tbl
from softarm.reports import img as _img
from softarm.labels import NDOSMC_KNOWN, SMC_KNOWN

OUT = ROOT / "docs" / "reports" / "smc-on-m2.html"
SMC, NDO = SMC_KNOWN, NDOSMC_KNOWN
L4E1, L4E10 = "SMC λ = 4, η = 1, no observer", "SMC λ = 4, η = 10, no observer"


THEORY = r"""
<section><h2>1 · The model the controllers are built on (M2)</h2>
<p>Each axis \(i \in \{x, y, z\}\) is handled separately. The input is the three segment commands \(u\) (S2–S4, psi);
the states are the measured segment pressures \(p\) and the tip \(x\) (mm from the settled pose).</p>
<div class="eqn">\[ \tau\,\dot p = -p + G\,u(t-T), \qquad \dot x_i = f_i(x_i) + b_i v_i, \qquad f_i(x_i) = -a_i x_i, \qquad v = H p \]</div>
<ul><li>\(T = 0.5\) s: transport delay (\(N = 50\) steps at 100 Hz). \(\tau \approx 0.6\) s: pneumatic lag. \(G\): command-to-pressure gain.</li>
<li>\(H\): pressure-to-tip map (3×3, identified from measured pressure). \(v = Hp\) is the "effective pressure" seen by each axis.</li>
<li>\(a_i = b_i = \Lambda_{ii}\), \(\Lambda = \mathrm{diag}(4.77,\ 4.70,\ 15.5)\) s⁻¹: each axis is a first-order lag towards \(H p\).</li></ul>
<p>Source: <code>docs/figures/task_space_model.json</code>, identified by <code>scripts/task_space_model.py</code>.</p></section>

<section><h2>2 · Seeing past the delay: the predicted tip \(\hat x\)</h2>
<p>A command sent now reaches the tip only after \(T\), so the law must act on where the tip <em>will</em> be.
The controller runs a delay-free copy of M2 (state \(x_m, p_m\)) driven by the commands it sends, and corrects the
measurement by the copy's motion over the last \(T\) (a Smith predictor):</p>
<div class="eqn">\[ \hat x(t) = x(t) + x_m(t) - x_m(t-T) \;\approx\; x(t+T) \]</div>
<p>If the copy were exact, \(\hat x(t) = x(t+T)\). The reference is known in advance, so the law compares
\(\hat x\) with \(x_d(t+T)\), the point the tip should reach when the command arrives.</p></section>

<section><h2>3 · Sliding-mode control on M2 (SMC)</h2>
<p>Tracking error and sliding variable:</p>
<div class="eqn">\[ E = \hat x - x_d(t+T), \qquad s = \lambda E + \dot E \]</div>
<p>Differentiate \(E\) and substitute the model:</p>
<div class="eqn">\[ \dot E = f(\hat x) + b\,v - \dot x_d(t+T) \]</div>
<p>Choose \(v\) to cancel the model and impose \(s = \tilde u\):</p>
<div class="eqn">\[ v = \frac{1}{b}\Big[ -f(\hat x) + \dot x_d(t+T) - \lambda E + \tilde u \Big], \qquad \tilde u = -\eta\,\mathrm{sat}\!\left(\frac{E}{\mu}\right) \]</div>
<p>\(\tilde u\) is the switching (robust) term; \(\mathrm{sat}\) is a boundary layer of width \(\mu\) that replaces
\(\mathrm{sgn}\) to avoid chattering. \(\hat x\) enters through \(f\) and \(E\), not through \(\tilde u\).
Substituting back gives the closed loop:</p>
<div class="eqn">\[ \dot E = -\lambda E - \eta\,\mathrm{sat}(E/\mu) \quad\Longrightarrow\quad s = \lambda E + \dot E = -\eta\,\mathrm{sat}(E/\mu) \]</div>
<ul><li>Outside the layer (\(|E| \ge \mu\)): \(\dot E = -\lambda E \mp \eta\), a reaching phase toward the layer.</li>
<li>Inside (\(|E| < \mu\)): \(\dot E = -(\lambda + \eta/\mu)\,E\), exponential decay at rate \(\lambda + \eta/\mu\).
This is the line the phase portraits below are compared with.</li>
<li>The code switches on \(E\), not on \(s\). Because M2's tip is first order, \(v\) already appears in \(\dot E\), so this
is the first-order analogue of the usual \(\ddot x\) form: \(\ddot x_d \to \dot x_d\), \(-\lambda \dot E \to -\lambda E\).</li></ul>
<p>Gains: \(\lambda = 8\ \mathrm{s^{-1}}\), \(\eta = \varepsilon = 1\ \mathrm{mm/s}\), \(\mu = 1\) mm.</p></section>

<section><h2>4 · From \(v\) to the valves</h2>
<div class="eqn">\[ p_\mathrm{des} = H^{-1} v, \qquad p_\mathrm{cmd} = p_\mathrm{des} + \tau\,\dot p_\mathrm{des}, \qquad u = 4.5 + G^{-1} p_\mathrm{cmd}, \ \ \mathrm{clipped\ to\ } [0, 9]\ \mathrm{psi} \]</div>
<p>The middle step inverts the pneumatic lag; 4.5 psi is the settled baseline. The clipped \(u\) is what drives the
internal M2 copy, so the predictor sees saturation.</p></section>

<section><h2>5 · Adding a disturbance observer (NDOSMC)</h2>
<p>Lump everything M2 does not model into a disturbance \(d\):</p>
<div class="eqn">\[ \dot x = f(x) + b\,v + d \]</div>
<p>The observer (paper eqs. 6–9, with \(l_2 = 0\)):</p>
<div class="eqn">\[ \hat d = z + l_1 \hat x, \qquad \dot z = l_1\big( a\,\hat x - b\,v_m - \hat d \big), \qquad v_m = H p_m \]</div>
<p>Differentiate \(\hat d\) and substitute \(\dot{\hat x} = -a\hat x + b v_m + d\):</p>
<div class="eqn">\[ \dot{\hat d} = \dot z + l_1 \dot{\hat x} = l_1\,(d - \hat d) \]</div>
<p>So \(\hat d\) is \(d\) passed through a first-order filter with time constant \(1/l_1\). With \(l_1 = 0.1\) that is
<b>10 s</b>: \(\hat d\) follows only the slow part of \(d\). The law subtracts it, and the switching gain becomes time
varying:</p>
<div class="eqn">\[ \tilde u = -\eta(t)\,\mathrm{sat}(E/\mu) - \hat d, \qquad \eta(t) = \Phi e^{-l_1 t} + \frac{\varepsilon}{l_1} \;\to\; \frac{\varepsilon}{l_1} = 10\ \mathrm{mm/s} \]</div>
<p>Everything else (\(\hat x\), \(E\), the valve chain) is as in the SMC. Nikoletta's gains: \(\lambda = 4\), \(l_1 = 0.1\),
\(l_2 = 0\), \(\mu = 1\) mm, \(\Phi = 1\), \(\varepsilon = 1\).</p></section>
"""


def main() -> None:
    d = json.loads((FIG / "smc_signals.json").read_text())
    runs = {(r["test"], r["controller"]): r for r in d["runs"]}
    tests = list(dict.fromkeys(r["test"] for r in d["runs"]))
    rms = lambda a: float(np.sqrt(np.mean(np.asarray(a) ** 2)))  # noqa: E731
    met = lambda t, n, k: runs[(t, n)]["metrics"][k]  # noqa: E731

    eta_rows = [[t] + [f"{met(t, n, 'rmse_whole_mm'):.2f} / {met(t, n, 'rmse_settled_mm'):.3f}" for n in (L4E1, L4E10, NDO)]
                for t in tests]
    eta = ('<section><h2>6 · What the observer adds</h2><p>The NDOSMC differs from the λ = 4 SMC in two ways: it subtracts '
           '\\(\\hat d\\), and its switching gain grows from \\(\\eta = 1\\) to \\(\\eta \\approx 10\\). Running the SMC at λ = 4 '
           'with η = 10 and no observer separates them. RMSE whole run / after settling (mm):</p>'
           + tbl(["Test", "SMC λ = 4, η = 1", "SMC λ = 4, η = 10", "NDOSMC (λ = 4, η → 10, d̂)"], eta_rows)
           + '<p>The η = 10 SMC without an observer matches the NDOSMC on every test. The improvement comes from the larger '
             'switching gain; \\(\\hat d\\) adds nothing measurable here. This is consistent with the next table: with a 10 s '
             'filter, \\(\\hat d\\) is a fraction of the actual disturbance.</p></section>')

    d_rows = []
    for t in tests:
        a, b = runs[(t, SMC)], runs[(t, NDO)]
        d_rows.append([t, f"{rms(np.array(a['d_mm_s'])[:, 0]):.2f} / {rms(np.array(a['d_mm_s'])[:, 1]):.2f}",
                       f"{rms(np.array(b['d_mm_s'])[:, 0]):.2f} / {rms(np.array(b['d_mm_s'])[:, 1]):.2f}",
                       f"{rms(np.array(b['d_hat_mm_s'])[:, 0]):.2f} / {rms(np.array(b['d_hat_mm_s'])[:, 1]):.2f}"])
    dist = ('<section><h2>7 · Pressures and disturbance on each test</h2>'
            '<p>The controller\'s states are the internal M2 copy \\((x_m, p_m)\\), the delay buffer and, in the NDOSMC, the '
            'observer state \\(z\\). The plots show what they produce and act on: the commanded pressure \\(u\\), the measured '
            'segment pressure \\(p\\), and the disturbance. The <b>actual</b> disturbance is the part of the measured motion M2 '
            'does not explain, \\(d = \\dot x - (-a x + b H (p - p_0))\\), from the tip velocity, tip and pouch sensors. '
            'It spikes at every corner and reference change (lag and coupling the first-order model misses). RMS over the run, '
            'x / y (mm/s):</p>'
            + tbl(["Test", "actual d, SMC run", "actual d, NDOSMC run", "observer d̂, NDOSMC"], d_rows) + '</section>')

    per_test = '<section><h2>9 · Each test</h2><p>Figures for every test, in the order of the comparison.</p></section>'
    for t in tests:
        f = t.replace(" ", "_")
        per_test += (f'<section><h3>{escape(t)}</h3><p class="meta">{escape(ABOUT[t])}</p>'
                     f'<figure><div class="scroll">{_img(f"smc_signals_{f}.png", t + ": pressures and disturbance")}</div>'
                     '<figcaption>Left: SMC (λ = 8). Right: NDOSMC. Rows: commanded pressure, measured pressure, disturbance '
                     '(dotted: actual d; solid: observer d̂).</figcaption></figure>'
                     f'<figure><div class="scroll">{_img(f"smc_phase_{f}.png", t + ": phase portrait")}</div>'
                     '<figcaption>Error-space phase portrait, E against Ė, x and y. Left: whole run, including the start-up '
                     'jump. Right: after settling. Dashed: sliding line s = 0. Dotted: the closed loop inside the boundary '
                     'layer, Ė = −(λ + η/μ)E.</figcaption></figure></section>')

    phase = r"""<section><h2>8 · Reading the phase portraits</h2><ul>
<li><b>Whole run:</b> each axis starts away from the origin (the reference starts away from the settled tip), travels
toward the lines (reaching phase), then slides into the origin along them.</li>
<li><b>Circles, after settling:</b> a small closed ellipse. It is the steady periodic error at the reference frequency;
its aspect ratio \(|\dot E|/|E| \approx \omega = 2\pi f\) (0.63 rad/s on the 0.1 Hz circle). With an exact model the
point would sit at the origin; the ellipse's size (hundredths of a mm) is the model error left over.</li>
<li><b>Triangle, square, waypoints:</b> at each corner \(\dot x_d\) jumps, so \(\dot E\) jumps vertically, and the state
then returns along the closed-loop line. The larger loops are the corner errors.</li>
<li>The state does not lie on \(s = 0\): this law makes \(s = -\eta\,\mathrm{sat}(E/\mu)\), so it follows the steeper
dotted line \(\dot E = -(\lambda + \eta/\mu)E\) instead. The NDOSMC's line is steeper (14 s⁻¹ against 9 s⁻¹) because
\(\eta \approx 10\).</li></ul></section>"""

    intro = ('<div class="wrap"><header><div class="eyebrow">Soft pneumatic arm · delay-aware sliding-mode control · reference known</div>'
             '<h1>Sliding-Mode Controllers on M2</h1><p class="meta">The SMC and the NDOSMC as implemented in '
             '<code>softarm.controllers.DelayAwareNDOSMC</code>, derived step by step, then their pressures, disturbance and error-space phase '
             'portraits on the six comparison tests. Regenerate: <code>uv run python scripts/smc_signals.py</code>, then '
             '<code>uv run python scripts/report_smc_explained.py</code>.</p></header>')
    html = head("Sliding-Mode Controllers on M2") + CSS + "\n" + intro + THEORY + eta + dist + phase + per_test + "</div>\n"
    OUT.write_text(html)
    print(f"wrote {OUT.relative_to(ROOT)} ({len(html) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
