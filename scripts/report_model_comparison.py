"""Build docs/reports/model-comparison.html from the results of compare_models_excitation.py.

Reads docs/figures/model_comparison.json and the model_* figures (embedded) and writes one
self-contained page: summary, the models, the test, error tables per excitation, static gains,
an overview figure and one figure per model, the actuator identification and the 0.5 s-ahead
prediction test (M2 with and without the disturbance observer). Best value per column is bold.

Usage:
    uv run python scripts/compare_models_excitation.py      # results and figures first
    uv run python scripts/report_model_comparison.py
"""

import base64
import json
from html import escape
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FIG = ROOT / "docs" / "figures"
OUT = ROOT / "docs" / "reports" / "model-comparison.html"

MODELS = ("one-mode", "one-mode+g", "M2", "M3", "F")
NAME = {"one-mode": "One-mode", "one-mode+g": "One-mode + gravity", "M2": "M2", "M3": "M3", "F": "F"}
SLUG = {"one-mode": "onemode", "one-mode+g": "onemode_gravity", "M2": "M2", "M3": "M3", "F": "F"}
KINDS = {"random_steps": "Random steps", "chirp": "Chirp"}
STATS = (("rmse_mm", "RMSE", "{:.3f}", lambda v: v), ("max_abs_mm", "max |e|", "{:.2f}", lambda v: v),
         ("bias_mm", "bias", "{:+.3f}", abs), ("fit_pct", "fit %", "{:.1f}", lambda v: -v))

CSS = """
:root { --bg:#f7f8fa; --surface:#ffffff; --fg:#1b1f24; --muted:#5a636e; --rule:#d9dde3; --accent:#0b5cad;
  --best:#e8f0fb; --display:"Archivo",system-ui,sans-serif; --body:"Source Sans 3",system-ui,sans-serif;
  --mono:"JetBrains Mono",ui-monospace,monospace; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { --bg:#111418; --surface:#181c21; --fg:#e4e8ec;
  --muted:#9aa4ae; --rule:#2c333b; --accent:#7fb6f0; --best:#1d2b3c; color-scheme:dark; } }
:root[data-theme="dark"] { --bg:#111418; --surface:#181c21; --fg:#e4e8ec; --muted:#9aa4ae; --rule:#2c333b;
  --accent:#7fb6f0; --best:#1d2b3c; color-scheme:dark; }
body { background:var(--bg); color:var(--fg); font:17px/1.55 var(--body);
  padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px)); }
main { max-width:1080px; margin:0 auto; padding-block:40px 80px; display:grid; gap:44px; }
h1, h2, h3 { font-family:var(--display); text-wrap:balance; line-height:1.2; margin:0; }
h1 { font-size:2.2rem; } h2 { font-size:1.45rem; padding-bottom:6px; border-bottom:1px solid var(--rule); }
h3 { font-size:1.08rem; margin-top:8px; }
section { display:grid; gap:14px; min-width:0; }
p, li { max-width:72ch; margin:0; } ul { margin:0; padding-left:1.2em; display:grid; gap:4px; }
.lede { color:var(--muted); font-size:1.08rem; }
code, .mono { font-family:var(--mono); font-size:.86em; }
.eq { font-family:var(--mono); font-size:.9rem; background:var(--surface); border:1px solid var(--rule);
  border-radius:6px; padding:10px 14px; overflow-x:auto; white-space:pre; }
.tablewrap { overflow-x:auto; border:1px solid var(--rule); border-radius:6px; background:var(--surface); }
table { border-collapse:collapse; width:100%; font-variant-numeric:tabular-nums; font-size:.92rem; }
th, td { padding:6px 10px; border-bottom:1px solid var(--rule); text-align:right; white-space:nowrap; }
th { font-family:var(--display); font-weight:600; font-size:.82rem; color:var(--muted); }
td:first-child, th:first-child { text-align:left; }
tr:last-child td { border-bottom:0; }
td.best { background:var(--best); font-weight:700; }
td.na { color:var(--muted); }
.group th { color:var(--fg); border-bottom:1px solid var(--rule); text-align:center; }
figure { margin:0; display:grid; gap:6px; min-width:0; }
figure img { width:100%; height:auto; background:#fff; border-radius:6px; border:1px solid var(--rule); }
figcaption { color:var(--muted); font-size:.92rem; max-width:80ch; }
.grid2 { display:grid; grid-template-columns:repeat(auto-fit, minmax(min(100%, 440px), 1fr)); gap:18px; }
.switch { display:flex; gap:6px; flex-wrap:wrap; }
.switch button { font:600 .9rem var(--display); padding:6px 14px; border-radius:999px; border:1px solid var(--rule);
  background:var(--surface); color:var(--fg); cursor:pointer; }
.switch button[aria-pressed="true"] { background:var(--accent); border-color:var(--accent); color:var(--bg); }
.switch button:focus-visible { outline:2px solid var(--accent); outline-offset:2px; }
.verdict { display:grid; grid-template-columns:repeat(auto-fit, minmax(min(100%, 300px), 1fr)); gap:14px; }
.verdict div { background:var(--surface); border:1px solid var(--rule); border-radius:6px; padding:12px 14px; display:grid; gap:4px; }
.verdict b { font-family:var(--display); }
.note { color:var(--muted); font-size:.92rem; }
"""


def img(name: str, alt: str, kind_attr: str = "") -> str:
    b = base64.b64encode((FIG / name).read_bytes()).decode()
    return f'<img {kind_attr} src="data:image/png;base64,{b}" alt="{escape(alt)}" loading="lazy">'


def error_table(res: dict) -> str:
    head = '<tr class="group"><th></th>' + "".join(f'<th colspan="4">{a}</th>' for a in "xyz") + "</tr><tr><th>model</th>"
    head += "".join(f"<th>{lab}</th>" for _ in "xyz" for _, lab, _, _ in STATS) + "</tr>"
    best = {}
    for a in "xyz":
        for key, _, fmt, score in STATS:
            vals = [score(float(fmt.format(res[m][a][key]))) for m in MODELS if res[m][a]]
            best[a, key] = min(vals)
    rows = ""
    for m in MODELS:
        rows += f"<tr><td>{NAME[m]}</td>"
        for a in "xyz":
            v = res[m][a]
            for key, _, fmt, score in STATS:
                if v is None:
                    rows += '<td class="na">–</td>'
                    continue
                txt = fmt.format(v[key])
                cls = ' class="best"' if score(float(txt)) == best[a, key] else ""
                rows += f"<td{cls}>{txt}</td>"
        rows += "</tr>"
    return f'<div class="tablewrap"><table>{head}{rows}</table></div>'


def gain_table(g: dict) -> str:
    head = "<tr><th>static gain (mm/psi)</th><th>Simulator</th>" + "".join(f"<th>{NAME[m]}</th>" for m in MODELS) + "</tr>"
    rows = ""
    for name, v in g.items():
        closest = min(MODELS, key=lambda m: abs(v[m] - v["truth"]))
        rows += f"<tr><td>{escape(name)}</td><td>{v['truth']:+.2f}</td>"
        for m in MODELS:
            cls = ' class="best"' if m == closest else ""
            rows += f"<td{cls}>{v[m]:+.2f} ({100 * v[m] / v['truth']:.0f} %)</td>"
        rows += "</tr>"
    return f'<div class="tablewrap"><table>{head}{rows}</table></div>'


def prediction_table(results: dict) -> str:
    names = list(results["random_steps"]["prediction_one_dead_time_ahead"])
    head = '<tr class="group"><th></th>' + "".join(f'<th colspan="3">{KINDS[k]}</th>' for k in KINDS) + "</tr>"
    head += "<tr><th>predictor</th>" + "".join(f"<th>{a}</th>" for _ in KINDS for a in "xyz") + "</tr>"
    rows = ""
    for n in names:
        rows += f"<tr><td>{escape(n)}</td>"
        for k in KINDS:
            p = results[k]["prediction_one_dead_time_ahead"]
            for a in "xyz":
                b = min(round(p[x][a], 3) for x in names)
                cls = ' class="best"' if round(p[n][a], 3) == b else ""
                rows += f"<td{cls}>{p[n][a]:.3f}</td>"
        rows += "</tr>"
    return f'<div class="tablewrap"><table>{head}{rows}</table></div>'


def switchable(base: str, alt: str, caption: str) -> str:
    """One figure per excitation; the page's switch shows one at a time."""
    imgs = "".join(img(base.format(kind=k), f"{alt}, {KINDS[k]}", f'data-kind="{k}"' + ("" if i == 0 else " hidden"))
                   for i, k in enumerate(KINDS))
    return f"<figure>{imgs}<figcaption>{caption}</figcaption></figure>"


def main() -> None:
    d = json.loads((FIG / "model_comparison.json").read_text())
    res, g, act = d["results"], d["static_gains_mm_per_psi"], d["actuator_identified_from_step"]
    rs, ch = res["random_steps"], res["chirp"]
    r = lambda k, m, a: res[k][m][a]["rmse_mm"]  # noqa: E731
    switch = '<div class="switch" role="group" aria-label="Excitation">' + "".join(
        f'<button type="button" data-show="{k}" aria-pressed="{str(i == 0).lower()}">{KINDS[k]}</button>'
        for i, k in enumerate(KINDS)) + "</div>"

    html = f"""<title>Soft Arm Model Comparison</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@600;700&family=JetBrains+Mono:wght@400;500&family=Source+Sans+3:wght@400;600;700&display=swap">
<style>{CSS}</style>
<main>
<header style="display:grid;gap:10px">
<h1>Soft arm model comparison</h1>
<p class="lede">Five models of the arm get the same pressure commands as the simulator
(<code>make_calibrated_sim</code>, seed 0) and run open loop from rest. Their tip paths are compared
with the simulator's on x, y and z, for two excitations of 60 s each.</p>
</header>

<section>
<h2>Summary</h2>
<div class="verdict">
<div><b>M3 is the most accurate</b><span>Free-run RMSE x / y: {r('random_steps','M3','x'):.2f} / {r('random_steps','M3','y'):.2f} mm on random steps,
{r('chirp','M3','x'):.2f} / {r('chirp','M3','y'):.2f} mm on the chirp. F is close behind; M2 is up to 3× worse on y in the chirp.</span></div>
<div><b>M2 and M3 are the only models with z</b><span>Both reach {r('random_steps','M2','z'):.2f} mm on random steps. On the chirp the z motion is tiny (under 0.3 mm) and both miss its shape (fit {ch['M2']['z']['fit_pct']:.0f} %).</span></div>
<div><b>The one-mode models are off by millimetres</b><span>One-mode: 7–14 mm RMSE, static gain {abs(g['x per psi on S3']['one-mode']):.1f} mm/psi vs the simulator's {abs(g['x per psi on S3']['truth']):.2f}. Adding gravity cuts the error about 3×, but with no dead time and no actuator lag the model still moves before the simulator does.</span></div>
<div><b>The disturbance observer does not improve the model's prediction</b><span>Predicting the tip 0.5 s ahead, M2 + NDO is slightly worse than M2 alone. The observer still helps the controller (see the last section).</span></div>
</div>
</section>

<section>
<h2>The models</h2>
<p>u = command − 4.5 psi on the three inputs S2, S3, S4 (S1 is sealed). y = tip position minus its
settled value, mm. Plane inputs for the per-axis models: ν<sub>x</sub> = −u<sub>3</sub>, ν<sub>y</sub> = u<sub>2</sub> − u<sub>4</sub>.</p>
<div class="tablewrap"><table>
<tr><th>model</th><th>equations</th><th>outputs</th><th>delay + lag</th></tr>
<tr><td>One-mode</td><td class="mono" style="text-align:left">ÿ + 9.231 ẏ + 10.60 y = b ν,  per axis</td><td>x, y</td><td>no</td></tr>
<tr><td>One-mode + gravity</td><td class="mono" style="text-align:left">ÿ + 9.409 ẏ + 51.45 y = b ν,  per axis</td><td>x, y</td><td>no</td></tr>
<tr><td>M2</td><td class="mono" style="text-align:left">τ ṗ = −p + G u(t − T);  ẏ = −a y + a H p</td><td>x, y, z</td><td>yes</td></tr>
<tr><td>M3</td><td class="mono" style="text-align:left">τ ṗ = −p + G u(t − T);  ÿ + 2ζω ẏ + ω² y = ω² H p</td><td>x, y, z</td><td>yes</td></tr>
<tr><td>F</td><td class="mono" style="text-align:left">state space per axis from frequency response, input ν delayed</td><td>x, y</td><td>yes</td></tr>
</table></div>
<p class="note">p: pouch pressure above its settled value (psi). T = 0.5 s dead time, τ = 0.6 s lag,
G ≈ 1.1 per input. H (3×3, mm/psi) maps the three pressures to x, y, z.</p>
</section>

<section>
<h2>The test</h2>
<ul>
<li><b>Inputs.</b> S2, S3, S4 push the tip toward +y, −x and −y. One excitation is a pressure trace for each input, played at the same time.</li>
<li><b>Random steps</b> (seed 31): each input jumps to an independent random level in 1.5–7.5 psi, held 0.5–3 s.</li>
<li><b>Chirp</b>: the same sine on all three inputs, shifted 0°, 120°, 240° so the push rotates around the arm; 4.5 ± 2.5 psi, frequency swept 0.02 → 1 Hz.</li>
<li><b>Free run.</b> No model ever sees the simulator's tip, so its errors accumulate the way they would inside a controller.</li>
<li><b>Metrics</b> per axis over the whole run: RMSE, largest |error| and bias = mean error (mm); fit = 100 (1 − ‖error‖ / ‖y − mean y‖) %. 100 % is perfect, 0 % is no better than a constant.</li>
</ul>
</section>

<section>
<h2>Errors</h2>
<figure>{img('model_comparison_rmse.png', 'Free-run RMSE per model and axis for both excitations, log scale')}
<figcaption>Free-run RMSE per model and axis, log scale. No bar: the model has no output on that axis.</figcaption></figure>
<h3>Random steps (mm; fit in %)</h3>
{error_table(rs)}
<h3>Chirp (mm; fit in %)</h3>
{error_table(ch)}
<p class="note">Highlighted: best in its column (smallest RMSE, |max|, |bias|; largest fit). –: no output on that axis.</p>
<h3>Static gain</h3>
<p>Tip change 10 s after a +1 psi step from 4.5 psi; percentage of the simulator in brackets.</p>
{gain_table(g)}
</section>

<section>
<h2>Tip paths</h2>
{switch}
<h3>All models together</h3>
{switchable('model_comparison_{kind}.png', 'Commands, then tip x, y, z for the simulator and all five models',
            'Top: the three commands. Below: tip x, y, z. Simulator: thick black line.')}
<h3>Each model against the simulator</h3>
<div class="grid2">
{''.join(switchable('model_comparison_{kind}_' + SLUG[m] + '.png', f'{NAME[m]} and the simulator, tip x, y, z',
                    f'{NAME[m]}: RMSE and fit in each panel title.') for m in MODELS)}
</div>
</section>

<section>
<h2>Actuator: dead time and lag from a step</h2>
<p>The actuator model is one first-order ODE plus a pure delay per input:</p>
<div class="eq">τ ṗ = −p + G u(t − T)        P(s)/U(s) = G e^(−sT) / (τ s + 1)
step Δu at t = 0:   p(t) = 0 for t &lt; T,   p(t) = G Δu (1 − e^(−(t − T)/τ)) for t ≥ T</div>
<p>T, τ and G can be identified without knowing them: step one input, record the measured pouch
pressure, take T as the first sample outside the noise band, then fit the step response by least
squares. A +1 psi step on S3 gives first rise at {act['dead_time_first_rise_s']:.2f} s and
<b>T = {act['dead_time_s']:.2f} s, τ = {act['tau_s']:.2f} s, G = {act['gain']:.2f}</b>, the values the models use.</p>
<figure>{img('model_actuator_step.png', 'Measured pouch pressure after a 1 psi step on S3 and the first-order fit')}
<figcaption>Measured pouch pressure (mean of the 5 pouches in the S3 column) and the fitted first-order response with dead time.</figcaption></figure>
<ul class="note">
<li>Without a step: for each candidate delay N fit p[k+1] = α p[k] + β u[k − N], keep the N with the smallest free-run error; τ = −Δt / ln α.</li>
<li>From a frequency sweep: phase = −ωT − arctan(ωτ), so the phase slope at high frequency gives T.</li>
<li>Without a pressure sensor the tip still shows T, but τ blends with the arm's own time constant.</li>
</ul>
</section>

<section>
<h2>Predicting 0.5 s ahead: M2 with and without the disturbance observer</h2>
<p>A disturbance observer estimates, from the measured tip, the part of the motion the model leaves
out: d in ẏ = −a y + a H p + d. With no measurement (a free run) it has nothing to estimate, so it is
scored on the job the delay-aware controllers give the model: predicting the tip one dead time ahead.</p>
<div class="eq">hold      ŷ(t+T) = y(t)
M2        ŷ(t+T) = y(t) + x_m(t) − x_m(t−T)        (Smith predictor; x_m = M2 without the dead time)
M2 + NDO  ŷ(t+T) = M2 + d̂ (1 − e^(−aT)) / a      d̂ = z + l₁ y,  ż = l₁ (a y − a H p̂ − d̂)</div>
<p class="note">The Smith predictor and the NDO already run together in the delay-aware NDOSMC: the predictor
gives the tip 0.5 s ahead, the observer estimates d on that predicted tip, and the sliding-mode law cancels d̂.
The question here is narrower: does d̂ also make the 0.5 s forecast better?</p>
<h3>Prediction RMSE (mm)</h3>
{prediction_table(res)}
<figure>{img('model_prediction_random_steps.png', 'Prediction error 0.5 s ahead for M2 and M2 plus NDO, and the observer estimate')}
<figcaption>Random steps. Left: prediction error, M2 vs M2 + NDO (l₁ = 0.1). Right: the observer's estimate d̂.</figcaption></figure>
<ul>
<li><b>It does not.</b> d̂ is a slow drift (x to about −0.3 mm/s, z to about 1.2 mm/s over 60 s): the gap between where
M2's equations say the tip settles and where the simulator settles. The Smith prediction starts from the measured tip,
so it already has that offset, and adding d̂ counts it twice.</li>
<li><b>It still helps the controller.</b> The sliding-mode law uses M2's equations directly, so the same offset
would leave a steady tracking error, and d̂ cancels it, like integral action. Same controller at λ = 4, observer off → on,
whole-run RMSE (mm): scored circle 1.286 → 1.206, held-out circle 2.047 → 1.948, triangle 1.069 → 1.012,
square 1.125 → 1.049, squiggle 0.361 → 0.308, random waypoints 0.363 → 0.274. After settling it is mixed
(scored circle 0.046 → 0.050, squiggle 0.086 → 0.095; better on the other four).</li>
</ul>
</section>

<section>
<h2>Reproduce</h2>
<div class="eq">uv run python scripts/compare_models_excitation.py
uv run python scripts/report_model_comparison.py</div>
<p class="note">Data: <code>docs/figures/model_comparison.json</code>, traces in <code>model_comparison_data.npz</code>.
Interactive version: <code>notebooks/model_check_excitation.ipynb</code>.</p>
</section>
</main>
<script>
document.querySelectorAll(".switch button").forEach(b => b.addEventListener("click", () => {{
  const k = b.dataset.show;
  document.querySelectorAll(".switch button").forEach(x => x.setAttribute("aria-pressed", String(x === b)));
  document.querySelectorAll("img[data-kind]").forEach(i => {{ i.hidden = i.dataset.kind !== k; }});
}}));
</script>
"""
    OUT.write_text(html)
    print(f"wrote {OUT.relative_to(ROOT)} ({OUT.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
