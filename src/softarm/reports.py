"""Shared pieces of the self-contained HTML reports in docs/reports (one page style for all).

    head(title)   <title>, fonts and MathJax for the page head
    CSS           the page style (light and dark)
    img(name)     a PNG from docs/figures embedded as a data URI
    tbl(head, rows)  a plain table, first column left-aligned, the others numeric
    ABOUT         one-line description of each comparison test (softarm.evaluation.TESTS)

Used by scripts/report_m2_controllers.py, report_smc_explained.py and report_mpc_explained.py.
"""

import base64
from html import escape

from softarm.constants import FIG

ABOUT = {
    "scored circle": "7 mm circle, 0.1 Hz (10 s per lap), sim seed 0; the course harness task.",
    "held-out circle": "10 mm circle, 0.15 Hz (6.7 s per lap), sim seed 5; never used for tuning.",
    "triangle": "Vertices on a 7 mm circle (first at +x), 10 s per lap, sim seed 0 (as motion_tests.py).",
    "square": "Axis-aligned, vertices on a 7 mm circle (side 9.9 mm), 10 s per lap, sim seed 0.",
    "squiggle": "Smooth random path: per axis 6 sines, random 0.03–0.3 Hz, ±6 mm, path seed 101; never repeats.",
    "random waypoints": "Straight lines between random points (±6 mm), 1–2.5 s per segment, path seed 101; sharp corners.",
}

CSS = """<style>
/* Layout: results report; one section per test with its table and figure. Best values: bold with a star (never colour alone). */
:root { --bg:#f6f6f3; --surface:#fff; --fg:#1c2124; --muted:#5d676c; --rule:#d8dcd9; --accent:#1f6f7a;
  --display:"Newsreader",Georgia,serif; --body:"IBM Plex Sans",system-ui,sans-serif; --mono:"IBM Plex Mono",ui-monospace,monospace; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { --bg:#121618; --surface:#1a2023; --fg:#e3e8ea; --muted:#9aa6ab; --rule:#2e383c; --accent:#6cc3cc; color-scheme:dark; } }
:root[data-theme="dark"] { --bg:#121618; --surface:#1a2023; --fg:#e3e8ea; --muted:#9aa6ab; --rule:#2e383c; --accent:#6cc3cc; color-scheme:dark; }
body { background:var(--bg); color:var(--fg); font:16px/1.6 var(--body); }
.wrap { max-width:1060px; margin:0 auto; padding-left:max(16px,env(safe-area-inset-left,0px)); padding-right:max(16px,env(safe-area-inset-right,0px)); padding-block:36px 72px; display:grid; gap:40px; }
header { display:grid; gap:10px; border-bottom:1px solid var(--rule); padding-bottom:22px; }
.eyebrow { font:500 12px var(--mono); letter-spacing:.08em; text-transform:uppercase; color:var(--muted); }
h1,h2 { font-family:var(--display); font-weight:600; margin:0; line-height:1.2; text-wrap:balance; } h1 { font-size:clamp(28px,5vw,40px); } h2 { font-size:25px; }
h3 { font:600 16px var(--body); margin:0; }
section { display:grid; gap:14px; min-width:0; } p,ul { margin:0; max-width:78ch; } ul { padding-left:22px; display:grid; gap:6px; }
.meta { color:var(--muted); font-size:14.5px; } code { font-family:var(--mono); font-size:.86em; }
.eqn { overflow-x:auto; background:var(--surface); border:1px solid var(--rule); border-radius:6px; padding:2px 14px; }
.tbl { overflow-x:auto; border:1px solid var(--rule); border-radius:6px; background:var(--surface); }
table { border-collapse:collapse; width:100%; font-size:14.5px; font-variant-numeric:tabular-nums; }
th,td { padding:7px 11px; text-align:left; border-bottom:1px solid var(--rule); vertical-align:top; } th { font-size:13px; color:var(--muted); font-weight:600; }
td.n,th.n { text-align:right; white-space:nowrap; } td:first-child { min-width:250px; } th.n { white-space:normal; min-width:84px; } td.best { font-weight:700; } tr:last-child td { border-bottom:0; }
figure { margin:0; display:grid; gap:10px; background:var(--surface); border:1px solid var(--rule); border-radius:6px; padding:12px; min-width:0; }
figure .scroll { overflow-x:auto; } figure .scroll > img { display:block; width:100%; min-width:760px; height:auto; background:#fff; border-radius:4px; }
figcaption { font-size:14.5px; color:var(--muted); }
</style>"""


def head(title: str) -> str:
    """Page title, web fonts and MathJax (\\( \\) inline, \\[ \\] display); CSS goes after it."""
    return (f'<title>{title}</title>\n<meta name="claude:viewport-fit-sides" content="cover">\n'
            '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&family=Newsreader:opsz,wght@6..72,600&display=swap">\n'
            '<script>window.MathJax={tex:{inlineMath:[["\\\\(","\\\\)"]],displayMath:[["\\\\[","\\\\]"]]},svg:{fontCache:"global"}};</script>\n'
            '<script src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/3.2.2/es5/tex-svg.js"></script>\n')


def img(name: str, alt: str) -> str:
    """docs/figures/<name> (PNG) as an <img> with a data URI."""
    b = base64.b64encode((FIG / name).read_bytes()).decode()
    return f'<img src="data:image/png;base64,{b}" alt="{escape(alt)}">'


def tbl(head: list, rows: list) -> str:
    """Cells are inserted as given (HTML allowed); columns after the first are right-aligned."""
    h = "".join(f'<th{"" if i == 0 else " class=n"}>{x}</th>' for i, x in enumerate(head))
    b = "".join("<tr>" + "".join(f'<td{"" if i == 0 else " class=n"}>{c}</td>' for i, c in enumerate(r)) + "</tr>"
                for r in rows)
    return f'<div class="tbl"><table><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table></div>'
