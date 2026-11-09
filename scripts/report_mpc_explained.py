"""Build docs/reports/mpc-on-m2-and-f.html: the two delay-compensated MPCs, derived and plotted.

Equations follow the code: compare_m2_controllers.M2MPC (MPC on M2) and
mpc_controller.DelayMPCController / AxisController (MPC on F). Numbers and figures come from
scripts/mpc_signals.py (docs/figures/mpc_signals.json, mpc_signals_<test>.png, mpc_phase_<test>.png).
Same page style as report_smc_explained.py.

Usage:
    uv run python scripts/mpc_signals.py          # results first
    uv run python scripts/report_mpc_explained.py
"""

import json
from html import escape

import numpy as np

import report_m2_controllers as R
from report_smc_explained import _img, tbl

FIG = R.FIG
OUT = R.ROOT / "docs" / "reports" / "mpc-on-m2-and-f.html"
M2, F = "MPC (M2), horizon 100 steps = 1.00 s", "MPC (F), horizon 300 steps = 3.00 s"

THEORY = r"""
<section><h2>1 · What both MPCs do every 10 ms</h2>
<ol><li>Estimate the current state from the measurements (tip, pouch pressures).</li>
<li>Predict across the 0.5 s dead time with the commands already sent (they cannot be changed).</li>
<li>Estimate a tip offset that the model does not explain.</li>
<li>Choose the next \(H\) commands that make the predicted tip follow the known reference \(r(k+N) \dots r(k+N+H-1)\)
with smooth commands. No constraints, so the optimum is a closed-form linear map. Apply only the first command, clip it
to 0–9 psi, and repeat next step.</li></ol>
<p>The difference between them is the model (M2 or F), the state estimate, and the offset.</p></section>

<section><h2>2 · MPC on M2: model and state</h2>
<p>State \(X = (y, p) \in \mathbb{R}^6\): tip \(y\) (mm) and measured S2–S4 pressure \(p\) (psi), both relative to the
settled pose. Input \(u\) = command − 4.5 psi. M2 in continuous time:</p>
<div class="eqn">\[ \dot X = \underbrace{\begin{bmatrix} -\Lambda & \Lambda H \\ 0 & -\tfrac{1}{\tau} I \end{bmatrix}}_{A_c} X
 + \underbrace{\begin{bmatrix} 0 \\ \tfrac{1}{\tau}\,\mathrm{diag}(G) \end{bmatrix}}_{B_c} u(t-T), \qquad y = C X = [\,I\ \ 0\,]\,X \]</div>
<p>Zero-order hold at \(\Delta t = 10\) ms gives \(A = e^{A_c\Delta t}\), \(B = \int_0^{\Delta t} e^{A_c s}B_c\,ds\), and</p>
<div class="eqn">\[ X_{k+1} = A X_k + B\,u_{k-N}, \qquad N = 50 \]</div>
<p>Both \(y\) and \(p\) are measured, so the state is read directly; no observer is needed.</p></section>

<section><h2>3 · MPC on M2: delay predictor and offset</h2>
<p>The commands \(u_{k-N} \dots u_{k-1}\) are already in the pipe. Rolling the model forward with them gives the state at
the moment the next command lands:</p>
<div class="eqn">\[ \hat X_{k+N} = A^N X_k + \sum_{i=0}^{N-1} A^{N-1-i} B\,u_{k-N+i} \]</div>
<p>The model is not exact. The offset \(w\) is the tip prediction error, measured now against the prediction made \(N\)
steps ago, and low-passed with a 0.5 s time constant:</p>
<div class="eqn">\[ w_k = w_{k-1} + \alpha\Big[\big(y_k - C\hat X_{k\,|\,k-N}\big) - w_{k-1}\Big], \qquad \alpha = 1 - e^{-\Delta t/0.5\,\mathrm{s}} \]</div>
<p>\(w\) is held constant over the horizon: an output disturbance added to every predicted tip.</p></section>

<section><h2>4 · MPC on M2: cost and solution</h2>
<p>Stack the predicted tips over the horizon and the free commands \(U = (u_k, \dots, u_{k+H-1})\):</p>
<div class="eqn">\[ \hat Y = \begin{bmatrix} C\hat X_{k+N+1} \\ \vdots \\ C\hat X_{k+N+H} \end{bmatrix} = \Phi\,\hat X_{k+N} + \Gamma\,U,
\qquad \Phi_i = C A^{i+1}, \qquad \Gamma_{ij} = C A^{i-j} B\ \ (j \le i) \]</div>
<p>Cost: tracking error of the offset-corrected prediction against the known reference, plus a penalty on command changes
(\(D\) is the first-difference matrix):</p>
<div class="eqn">\[ J = \sum_{i=0}^{H-1} \big\| C\hat X_{k+N+1+i} + w - r_{k+N+i} \big\|^2 + R_u \sum_{i=0}^{H-1} \big\| u_{k+i} - u_{k+i-1} \big\|^2
 = \big\|\hat Y + \mathbf{1}\otimes w - \mathcal R\big\|^2 + R_u\big\|DU - e_1 u_{k-1}\big\|^2 \]</div>
<p>Setting \(\partial J/\partial U = 0\):</p>
<div class="eqn">\[ U^\star = \big(\Gamma^\top\Gamma + R_u D^\top D\big)^{-1}\Big[\Gamma^\top\big(\mathcal R - \Phi\hat X_{k+N} - \mathbf 1\otimes w\big) + R_u D^\top e_1\,u_{k-1}\Big] \]</div>
<p>Only the first block row is used. It is a fixed linear law, computed once:</p>
<div class="eqn">\[ u_k = g_r\,\mathcal R + g_x\,\hat X_{k+N} + g_w\,w + g_u\,u_{k-1}, \qquad u_\mathrm{cmd} = \mathrm{clip}_{[0,9]}(4.5 + u_k) \]</div>
<ul><li>\(g_r\): feedforward over the whole reference window, so corners are seen up to \(H\) steps after the dead time.</li>
<li>\(g_x, g_w\): feedback on the predicted state and the offset. \(g_u\): smoothing from the command-change penalty.</li>
<li>Horizon \(H\) = 100 steps = 1 s after the 0.5 s dead time (looks 0.5–1.5 s ahead). \(R_u = 10^{-5}\) mm²/psi², tuned on a
separate 5 mm, 0.07 Hz circle. That weight is almost zero, so the law is close to an inverse of M2.</li>
<li>The clipped command, not \(u_k\), goes into the predictor's buffer, so saturation is accounted for.</li></ul>
<p>Code: <code>scripts/compare_m2_controllers.py</code>, class <code>M2MPC</code>.</p></section>

<section><h2>5 · MPC on F: per-channel model and Kalman filter</h2>
<p>F treats the planar motion as two independent channels, each driven by one combination of segment commands:</p>
<div class="eqn">\[ \nu_x = -u_3, \qquad \nu_y = u_2 - u_4, \qquad x^{(j)}_{n+1} = A_d x^{(j)}_n + B_d\,\nu^{(j)}_{n-d}, \qquad y^{(j)} = C x^{(j)} + o^{(j)} \]</div>
<p>\(A_d, B_d, C\) come from frequency-response identification (<code>scripts/sysid_frf.py</code>,
<code>models/sysid_calibrated_v2.json</code>); \(d = 50\) samples; the first state is the channel pressure. A steady-state
Kalman filter estimates the augmented state \(z = (x, o)\) from two measurements, tip and channel pressure:</p>
<div class="eqn">\[ z \leftarrow z + L\big(m - \mathcal H z\big), \qquad m = \begin{bmatrix} y \\ p_\mathrm{ch} \end{bmatrix}, \qquad
\mathcal H = \begin{bmatrix} C & 1 \\ e_1^\top & 0 \end{bmatrix}, \qquad p_{\mathrm{ch},x} = -\overline{S3},\ \ p_{\mathrm{ch},y} = \overline{S2 - S4} \]</div>
<p>\(L\) comes from the discrete Riccati equation; the offset \(o\) is modelled as a slow random walk, so the filter
assigns persistent tip error to it. Then the same predictor and cost as on M2, per channel, with the offset \(o\) in
place of \(w\):</p>
<div class="eqn">\[ \hat x_{n+d} = A_d^d x_n + \sum_{i=0}^{d-1} A_d^{d-1-i} B_d\,\nu_{n-d+i}, \qquad
J = \sum_{i=0}^{H-1}\big(C\hat x_{n+d+1+i} + o - r_{n+d+i}\big)^2 + \rho\sum_{i=0}^{H-1}\big(\Delta\nu_{n+i}\big)^2 \]</div>
<p>\(H\) = 300 steps = 3 s, \(\rho = 10^{-5}\) m²/psi² (its own tuning). The two channel commands go back to segments:</p>
<div class="eqn">\[ S2 = 4.5 + \tfrac{\nu_y}{2}, \qquad S3 = 4.5 - \nu_x, \qquad S4 = 4.5 - \tfrac{\nu_y}{2}, \qquad \mathrm{clipped\ to\ } [0, 9] \]</div>
<p>Code: <code>scripts/mpc_controller.py</code>, classes <code>AxisController</code> and <code>DelayMPCController</code>.</p></section>

<section><h2>6 · How the MPCs differ from the SMCs</h2><ul>
<li><b>Look-ahead:</b> the SMC uses \(x_d(t+T)\) and \(\dot x_d(t+T)\); the MPC uses the whole window
\(x_d(t+T) \dots x_d(t+T+H\Delta t)\), so it starts turning before a corner arrives.</li>
<li><b>Disturbance:</b> the NDOSMC estimates an input disturbance \(\hat d\) (mm/s) through a 10 s filter; the MPCs
estimate an output offset (mm): M2 through a 0.5 s low-pass of its prediction error, F through the Kalman filter.</li>
<li><b>Robustness term:</b> the SMC has a switching term \(-\eta\,\mathrm{sat}(E/\mu)\); the MPC has none. Its feedback is
linear, with fixed gains.</li>
<li><b>Both</b> run a delay predictor on the same principle; the SMC's is the Smith form \(x + x_m(t) - x_m(t-T)\).</li></ul></section>
"""

READING = r"""<section><h2>8 · Reading the plots</h2><ul>
<li><b>Commanded pressure:</b> the MPC on M2 jitters, because \(R_u = 10^{-5}\) makes it nearly invert M2 and pass
measurement-level variations straight to the valves. The MPC on F, with a 3 s horizon and a Kalman filter, is smooth.</li>
<li><b>Actual disturbance:</b> the same definition as on the SMC page, \(d = \dot x - (-a x + b H(p - p_0))\), measured
against M2. It spikes at corners in both runs.</li>
<li><b>Offset:</b> M2's \(w\) spikes at corners and decays (0.5 s low-pass); F's \(o\) moves slowly with the
lap and is larger on x than on y on every test (the x channel is driven by S3 alone).</li>
<li><b>Phase portrait:</b> \(E\) is the error the MPC aims to remove first, the offset-corrected predicted tip at
arrival minus the reference then, \(E = C\hat X_{k+N} + w - r_{k+N}\). There is no sliding line: the MPC has no
switching surface. The whole-run panels show the start-up jump being closed.</li>
<li><b>Predicted vs real error:</b> on the triangle, the MPC on M2 keeps its own predicted error within about ±0.07 mm
after settling, smaller than F's (±0.25 mm), yet its real tracking error is larger on cornered paths (triangle 0.129 vs 0.066 mm). The gap is
model error that \(w\) only partly catches: M2 believes it is on track. F's better model and estimator close more of it.</li></ul></section>"""


def main() -> None:
    d = json.loads((FIG / "mpc_signals.json").read_text())
    runs = {(r["test"], r["controller"]): r for r in d["runs"]}
    tests = list(dict.fromkeys(r["test"] for r in d["runs"]))
    rms = lambda a: np.sqrt(np.mean(np.asarray(a) ** 2, axis=0))  # noqa: E731

    rows = []
    for t in tests:
        row = [t]
        for n in (M2, F):
            r = runs[(t, n)]
            dd, oo = rms(r["d_mm_s"]), rms(r["offset_mm"])
            row += [f"{r['metrics']['rmse_settled_mm']:.3f}", f"{dd[0]:.2f} / {dd[1]:.2f}", f"{oo[0]:.3f} / {oo[1]:.3f}"]
        rows.append(row)
    numbers = ('<section><h2>7 · Numbers per test</h2><p>RMSE after settling (mm, real tracking error), RMS of the actual '
               'disturbance (mm/s, x / y) and RMS of the MPC\'s offset estimate (mm, x / y).</p>'
               + tbl(["Test", "M2: RMSE", "M2: d", "M2: offset w", "F: RMSE", "F: d", "F: offset o"], rows) + '</section>')

    per_test = '<section><h2>9 · Each test</h2><p>Figures for every test, in the order of the comparison.</p></section>'
    for t in tests:
        f = t.replace(" ", "_")
        per_test += (f'<section><h3>{escape(t)}</h3><p class="meta">{escape(R.ABOUT[t])}</p>'
                     f'<figure><div class="scroll">{_img(f"mpc_signals_{f}.png", t + ": MPC pressures and disturbance")}</div>'
                     '<figcaption>Left: MPC on M2 (horizon 100 steps = 1 s). Right: MPC on F (300 steps = 3 s). Rows: commanded '
                     'pressure, measured pressure, actual disturbance d, the MPC\'s own offset estimate.</figcaption></figure>'
                     f'<figure><div class="scroll">{_img(f"mpc_phase_{f}.png", t + ": MPC phase portrait")}</div>'
                     '<figcaption>Error-space phase portrait, E against Ė, x and y. Left: whole run. Right: after settling.'
                     '</figcaption></figure></section>')

    head = ('<title>MPC on M2 and F</title>\n<meta name="claude:viewport-fit-sides" content="cover">\n'
            '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&family=Newsreader:opsz,wght@6..72,600&display=swap">\n'
            '<script>window.MathJax={tex:{inlineMath:[["\\\\(","\\\\)"]],displayMath:[["\\\\[","\\\\]"]]},svg:{fontCache:"global"}};</script>\n'
            '<script src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/3.2.2/es5/tex-svg.js"></script>\n')
    intro = ('<div class="wrap"><header><div class="eyebrow">Soft pneumatic arm · delay-compensated MPC · reference known</div>'
             '<h1>MPC on M2 and F</h1><p class="meta">The two model-predictive controllers of the comparison, derived from '
             'the code step by step, then their pressures, disturbance and error-space phase portraits on the six tests. '
             'Companion to the SMC page. Regenerate: <code>uv run python scripts/mpc_signals.py</code>, then '
             '<code>uv run python scripts/report_mpc_explained.py</code>.</p></header>')
    html = head + R.CSS + "\n" + intro + THEORY + numbers + READING + per_test + "</div>\n"
    OUT.write_text(html)
    print(f"wrote {OUT.relative_to(R.ROOT)} ({len(html) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
