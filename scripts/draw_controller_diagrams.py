"""Write draw.io block diagrams of the controllers in compare_m2_controllers.py.

One page per controller, drawn from the code (scripts/ndosmc.py DelayAwareNDOSMC,
scripts/compare_m2_controllers.py M2MPC, scripts/mpc_controller.py DelayMPCController):
  1. Delay-aware SMC, reference known (lambda = 8 and the lambda = 4 row; observer off)
  2. Delay-aware NDOSMC, reference known (observer on)
  3. Delay-aware SMC, Taylor preview (reference not known)
  4. MPC on M2, reference known
  5. MPC on F, reference known
Blue blocks run in the controller every 10 ms, dashed blocks are models the controller runs
internally, grey is the simulator. Signals are in mm (tip) and psi (pressure).

Writes docs/diagrams/controllers.drawio (open with draw.io / diagrams.net, or the VS Code
Draw.io Integration extension).

Usage:
    uv run python scripts/draw_controller_diagrams.py
"""

from html import escape
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "docs" / "diagrams" / "controllers.drawio"

BASE = "rounded=1;whiteSpace=wrap;html=1;fontSize=13;arcSize=8;"
CTRL = BASE + "fillColor=#DCEAF7;strokeColor=#1F4E79;"
MODEL = BASE + "fillColor=#FFFFFF;strokeColor=#1F4E79;dashed=1;"
PLANT = BASE + "fillColor=#E6E6E6;strokeColor=#333333;fontStyle=0;"
TEXT = "text;html=1;whiteSpace=wrap;align=center;verticalAlign=middle;fontSize=13;"
NOTE = "text;html=1;whiteSpace=wrap;align=left;verticalAlign=top;fontSize=12;fontColor=#333333;"
TITLE = "text;html=1;align=left;verticalAlign=middle;fontSize=20;fontStyle=1;"
SUM = "ellipse;whiteSpace=wrap;html=1;fontSize=16;fillColor=#FFFFFF;strokeColor=#1F4E79;"
GROUP = ("rounded=1;whiteSpace=wrap;html=1;fillColor=none;strokeColor=#7F7F7F;dashed=1;dashPattern=8 4;"
         "verticalAlign=top;align=left;spacingLeft=8;fontSize=12;fontColor=#555555;arcSize=3;")
EDGE = ("edgeStyle=orthogonalEdgeStyle;rounded=0;html=1;endArrow=block;endFill=1;strokeWidth=1.5;"
        "fontSize=12;jumpStyle=arc;jumpSize=10;labelBackgroundColor=#FFFFFF;")
PORT = {"l": (0, 0.5), "r": (1, 0.5), "t": (0.5, 0), "b": (0.5, 1)}


class Page:
    def __init__(self, name: str, title: str):
        self.name, self.cells, self.n = name, [], 0
        self.box("title", title, 20, 10, 1500, 40, TITLE)

    def _id(self) -> str:
        self.n += 1
        return f"c{self.n}"

    def box(self, key: str, label: str, x, y, w, h, style=CTRL) -> str:
        self.cells.append(f'<mxCell id="{key}" value="{escape(label)}" style="{style}" vertex="1" parent="1">'
                          f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>')
        return key

    def sum(self, key: str, x, y) -> str:
        return self.box(key, "Σ", x, y, 44, 44, SUM)

    def edge(self, a: str, b: str, label: str = "", out: str | tuple = "r", inn: str | tuple = "l",
             pts: tuple = (), style: str = "", lx: float = 0.0) -> None:
        ox, oy = PORT.get(out, out) if isinstance(out, str) else out
        ix, iy = PORT.get(inn, inn) if isinstance(inn, str) else inn
        s = EDGE + f"exitX={ox};exitY={oy};exitDx=0;exitDy=0;entryX={ix};entryY={iy};entryDx=0;entryDy=0;" + style
        geo = '<mxGeometry relative="1" as="geometry"'
        geo += f' x="{lx}">' if lx else ">"
        if pts:
            geo += "<Array as=\"points\">" + "".join(f'<mxPoint x="{px}" y="{py}"/>' for px, py in pts) + "</Array>"
        geo += "</mxGeometry>"
        self.cells.append(f'<mxCell id="{self._id()}" value="{escape(label)}" style="{s}" edge="1" parent="1" '
                          f'source="{a}" target="{b}">{geo}</mxCell>')

    def xml(self) -> str:
        body = "".join(self.cells)
        return (f'<diagram name="{escape(self.name)}" id="{escape(self.name.replace(" ", "_"))}">'
                '<mxGraphModel dx="1600" dy="900" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" '
                'arrows="1" fold="1" page="1" pageScale="1" pageWidth="1654" pageHeight="900" math="0" shadow="0">'
                f'<root><mxCell id="0"/><mxCell id="1" parent="0"/>{body}</root></mxGraphModel></diagram>')


LEGEND = ("Blue: computed by the controller every 10 ms. Dashed: model run inside the controller. "
          "Grey: the simulator (make_calibrated_sim). Tip q in mm from the settled pose; pressures in psi.")
PLANT_LABEL = ("<b>Plant</b>: make_calibrated_sim<br>dead time T = 0.5 s<br>→ pneumatic lag τ ≈ 0.6 s<br>"
               "→ pouches → MuJoCo arm")


def smc_page(name: str, title: str, preview: str, observer: bool, note: str) -> Page:
    """Delay-aware (ND)OSMC: look-ahead or Taylor preview, Smith predictor on M2, lag inverse."""
    p = Page(name, title)
    p.box("grp", "Controller (every 10 ms, per axis x, y, z)", 110, 70, 1300, 560, GROUP)
    if preview == "known":
        p.box("ref", "Known trajectory<br>r(t), all t", 150, 95, 150, 60, TEXT)
        p.box("look", "<b>Look-ahead</b><br>r(t+T), ṙ(t+T) read from the trajectory<br>T = 0.5 s (N = 50 steps)",
              480, 90, 290, 70)
    else:
        p.box("ref", "Reference r(t) only<br>(future unknown)", 150, 95, 150, 60, TEXT)
        p.box("look", "<b>Taylor preview</b><br>r̂(t+T) = r + T ṙ + ½T² r̈<br>ṙ, r̈: backward differences of r",
              480, 90, 290, 70)
    p.edge("ref", "look")
    d_term = " − d̂" if observer else ""
    eta = "η = Φ e<sup>−l₁t</sup> + ε / l₁" if observer else "η = ε"
    p.box("law", f"<b>Sliding-mode law</b><br>σ = q̂ − r(t+T)<br>v = [ṙ(t+T) + a q̂ − λσ − η sat(σ/μ){d_term}] / b"
          f"<br>{eta}", 450, 200, 350, 110)
    p.edge("look", "law", "r(t+T), ṙ(t+T)", "b", "t")
    p.box("alloc", "<b>Allocation</b><br>p<sub>des</sub> = H⁻¹ v", 860, 220, 120, 70)
    p.box("laginv", "<b>Lag inverse</b><br>p<sub>cmd</sub> = p<sub>des</sub> + τ ṗ<sub>des</sub>", 1050, 220, 140, 70)
    p.box("cmd", "<b>Command</b><br>u = 4.5 + p<sub>cmd</sub> / G<br>clip 0–9 psi", 1240, 215, 150, 80)
    p.box("plant", PLANT_LABEL, 1480, 200, 200, 110, PLANT)
    p.box("out", "tip q", 1700, 235, 50, 40, TEXT)
    p.edge("law", "alloc", "v")
    p.edge("alloc", "laginv", "p<sub>des</sub>")
    p.edge("laginv", "cmd")
    p.edge("cmd", "plant", "u (S2–S4)")
    p.edge("plant", "out")
    p.box("model", "<b>Delay-free M2</b> (Smith predictor)<br>τ ṗ<sub>m</sub> = −p<sub>m</sub> + G (u − 4.5)<br>"
          "ẋ<sub>m</sub> = −a x<sub>m</sub> + b H p<sub>m</sub>", 1050, 380, 290, 90, MODEL)
    p.edge("cmd", "model", "u sent", "b", (0.9, 0))
    p.box("delay", "<b>Delay</b> z<sup>−N</sup><br>N = 50", 720, 480, 120, 60, MODEL)
    p.sum("sum", 603, 380)
    p.edge("model", "sum", "+ x<sub>m</sub>(t)", "l", "r")
    p.edge("model", "delay", "", (0.3, 1), "r", pts=((1137, 510),))
    p.edge("delay", "sum", "− x<sub>m</sub>(t−T)", "l", "b", pts=((625, 510),))
    p.edge("plant", "sum", "+ q (measured tip)", "b", "l", pts=((1580, 600), (560, 600), (560, 402)), lx=-0.3)
    p.edge("sum", "law", "q̂ = q + x<sub>m</sub>(t) − x<sub>m</sub>(t−T)  ≈ q(t+T)", "t", (0.5, 1), lx=0)
    if observer:
        p.box("ndob", "<b>Disturbance observer</b> (NDOB)<br>d̂ = z + l₁ q̂<br>ż = l₁ (a q̂ − b v<sub>m</sub> − d̂)",
              150, 360, 260, 90)
        p.edge("sum", "ndob", "q̂", (0, 0), "r", pts=((570, 370), (570, 405)))
        p.edge("model", "ndob", "v<sub>m</sub> = H p<sub>m</sub>", (0.1, 1), "b",
               pts=((1079, 570), (280, 570)))
        p.edge("ndob", "law", "d̂", "t", "l", pts=((280, 255),))
    p.box("note", note + "<br><br>" + LEGEND, 110, 650, 1300, 110, NOTE)
    return p


def mpc_m2_page() -> Page:
    p = Page("MPC on M2", "MPC on M2, reference known")
    p.box("grp", "Controller (every 10 ms; state X = (y, p) ∈ ℝ⁶, input u ∈ ℝ³)", 60, 70, 1250, 560, GROUP)
    p.box("ref", "Known trajectory<br>r(k), all k", 560, 85, 150, 50, TEXT)
    p.box("win", "<b>Reference window</b><br>r(k+N) … r(k+N+H−1)<br>H = 25 / 100 / 300 steps = 0.25 / 1 / 3 s",
          760, 80, 320, 70)
    p.edge("ref", "win")
    p.box("mpc", "<b>Unconstrained MPC</b> (closed form, first move only)<br>"
          "min Σ<sub>i</sub> ‖C X(k+N+1+i) + w − r(k+N+i)‖² + R<sub>u</sub> Σ<sub>i</sub> ‖Δu(k+i)‖²<br>"
          "u(k) = g<sub>r</sub> r + g<sub>x</sub> X(k+N) + g<sub>w</sub> w + g<sub>u</sub> u(k−1)",
          640, 190, 470, 110)
    p.edge("win", "mpc", "r(k+N … k+N+H−1)", "b", (0.75, 0))
    p.box("cmd", "<b>Command</b><br>u<sub>cmd</sub> = 4.5 + u(k)<br>clip 0–9 psi", 1150, 205, 140, 80)
    p.box("plant", PLANT_LABEL, 1360, 190, 200, 110, PLANT)
    p.box("out", "tip q", 1580, 225, 50, 40, TEXT)
    p.edge("mpc", "cmd")
    p.edge("cmd", "plant", "u (S2–S4)")
    p.edge("plant", "out")
    p.box("state", "<b>Measured state</b> X(k)<br>y = tip − settled tip (mm)<br>"
          "p = S2–S4 segment pressure − settled (psi)", 90, 380, 270, 90)
    p.box("pred", "<b>Delay predictor</b> (M2, zero-order hold, 10 ms)<br>"
          "X(k+N) = A<sup>N</sup> X(k) + Σ<sub>i</sub> A<sup>N−1−i</sup> B u(k−N+i)<br>N = 50", 480, 380, 380, 90)
    p.box("buf", "<b>Command buffer</b><br>u(k−N) … u(k−1)<br>(clipped, as applied)", 960, 385, 200, 80, MODEL)
    p.box("off", "<b>Model-error offset</b><br>w ← w + α [(y(k) − ŷ(k | k−N)) − w]<br>"
          "ŷ(k | k−N) = C X(k) predicted N steps ago<br>α: low-pass, 0.5 s", 90, 190, 320, 100)
    p.edge("state", "pred", "X(k)")
    p.edge("buf", "pred", "", "l", "r")
    p.edge("cmd", "buf", "u(k) applied", "b", (0.75, 0), pts=((1220, 340), (1110, 340)))
    p.edge("pred", "mpc", "X(k+N)", (0.6, 0), (0.25, 1), pts=((708, 340), (757, 340)))
    p.edge("state", "off", "y(k)", (0.3, 0), (0.3, 1))
    p.edge("pred", "off", "C X(k+N), stored", (0.1, 0), (0.85, 1), pts=((518, 330), (362, 330)))
    p.edge("off", "mpc", "w", "r", "l")
    p.edge("plant", "state", "tip, segment pressures (S2–S4)", "b", "b",
           pts=((1460, 580), (225, 580)), lx=-0.3)
    p.box("note", "Code: scripts/compare_m2_controllers.py M2MPC. M2: τ ṗ = −p + G u(t−T), ẏ = −Λ y + Λ H p "
          "(docs/figures/task_space_model.json). R<sub>u</sub> tuned per horizon on a 5 mm, 0.07 Hz circle "
          "(chosen 10⁻⁵ mm²/psi²; also run at R<sub>u</sub> = 1 and 10). Clipping is not in the optimisation; "
          "the clipped command enters the buffer.<br><br>" + LEGEND, 60, 650, 1250, 110, NOTE)
    return p


def mpc_f_page() -> Page:
    p = Page("MPC on F", "MPC on F (frequency-identified model), reference known")
    p.box("grp", "Controller (every 10 ms), one copy per planar channel: x and y; model F per channel "
          "(models/sysid_calibrated_v2.json)", 60, 70, 1250, 560, GROUP)
    p.box("ref", "Known trajectory<br>r(k), all k", 560, 85, 150, 50, TEXT)
    p.box("win", "<b>Reference window</b><br>r(k+d) … r(k+d+H−1)<br>H = 300 steps = 3 s, d = 50",
          760, 80, 300, 70)
    p.edge("ref", "win")
    p.box("mpc", "<b>Unconstrained MPC</b> (closed form, first move only)<br>"
          "min Σ<sub>i</sub> (C x(k+d+1+i) + o − r(k+d+i))² + ρ Σ<sub>i</sub> Δu(k+i)²<br>"
          "ρ = 10⁻⁵ m²/psi²", 640, 190, 440, 100)
    p.edge("win", "mpc", "r(k+d … k+d+H−1)", "b", (0.75, 0))
    p.box("map", "<b>Channels → segments</b><br>S2 = 4.5 + u<sub>y</sub>/2<br>S3 = 4.5 − u<sub>x</sub><br>"
          "S4 = 4.5 − u<sub>y</sub>/2<br>clip 0–9 psi", 1120, 180, 160, 120)
    p.box("plant", PLANT_LABEL, 1360, 185, 200, 110, PLANT)
    p.box("out", "tip q", 1580, 220, 50, 40, TEXT)
    p.edge("mpc", "map", "u<sub>x</sub>, u<sub>y</sub>")
    p.edge("map", "plant", "S2–S4")
    p.edge("plant", "out")
    p.box("chan", "<b>Channel pressures</b><br>p<sub>x</sub> = −mean(S3 pouches)<br>"
          "p<sub>y</sub> = mean(S2 − S4 pouches)<br>(5 pouches each)", 90, 400, 220, 100)
    p.box("kf", "<b>Kalman filter</b> on [x; tip offset o]<br>measurements: tip, channel pressure<br>"
          "x<sub>1</sub> = channel pressure", 350, 400, 270, 100)
    p.box("pred", "<b>Delay predictor</b><br>x(k+d) = A<sub>d</sub><sup>d</sup> x(k) + Σ<sub>i</sub> "
          "A<sub>d</sub><sup>d−1−i</sup> B<sub>d</sub> u(k−d+i)", 660, 405, 300, 90)
    p.box("buf", "<b>Command buffer</b><br>applied channel commands<br>u<sub>x</sub> = 4.5 − S3, "
          "u<sub>y</sub> = S2 − S4", 1010, 405, 220, 90, MODEL)
    p.edge("chan", "kf", "p<sub>x</sub>, p<sub>y</sub>")
    p.edge("kf", "pred", "x(k)")
    p.edge("buf", "pred", "", "l", "r")
    p.edge("buf", "kf", "u(k−d) (time update)", "b", "b", pts=((1120, 540), (485, 540)))
    p.edge("map", "buf", "applied (after clip)", "b", (0.75, 0), pts=((1200, 350), (1175, 350)))
    p.edge("pred", "mpc", "x(k+d)", "t", (0.3, 1), pts=((810, 340), (772, 340)))
    p.edge("kf", "mpc", "offset o", "t", (0.05, 1), pts=((485, 340), (662, 340)))
    p.edge("plant", "chan", "pouch pressures", "b", "b", pts=((1460, 590), (200, 590)), lx=-0.4)
    p.edge("plant", "kf", "tip x, y", (0.2, 1), (0.25, 1), pts=((1400, 570), (417, 570)), lx=-0.3)
    p.box("note", "Code: scripts/mpc_controller.py DelayMPCController(preview=\"oracle\") with the true "
          "future reference, as built in compare_m2_controllers.mpc_f. F: per-channel state-space model "
          "x<sub>n+1</sub> = A<sub>d</sub> x<sub>n</sub> + B<sub>d</sub> u<sub>n−d</sub> identified from "
          "frequency responses (scripts/sysid_frf.py). Unlike M2, F does not model z and treats x and y "
          "as independent channels.<br><br>" + LEGEND, 60, 650, 1250, 110, NOTE)
    return p


def main() -> None:
    pages = [
        smc_page("SMC known ref", "Delay-aware SMC (M2), reference known",
                 "known", False,
                 "Code: scripts/ndosmc.py DelayAwareNDOSMC(observer=False, exact_preview=True), fed r(t+T) by "
                 "benchmark_slide_tasks.Ahead. Rows: λ = 8 s⁻¹ and λ = 4 s⁻¹; μ = 1 mm, ε = 1. "
                 "a, b, H, G, τ from M2 (docs/figures/task_space_model.json); the observer is off (d̂ = 0)."),
        smc_page("NDOSMC known ref", "Delay-aware NDOSMC (M2), reference known",
                 "known", True,
                 "Code: scripts/ndosmc.py DelayAwareNDOSMC(exact_preview=True) with the observer on. "
                 "Nikoletta's gains: λ = 4 s⁻¹, l₁ = 0.1, l₂ = 0, μ = 1 mm, Φ = 1, ε = 1. Same structure as the "
                 "SMC page plus the observer, which acts on the predicted tip q̂."),
        smc_page("SMC Taylor preview", "Delay-aware SMC (M2), Taylor preview (reference not known)",
                 "taylor", False,
                 "Code: scripts/ndosmc.py DelayAwareNDOSMC(observer=False), λ = 8 s⁻¹, μ = 1 mm, ε = 1. "
                 "Identical to the known-reference SMC except the look-ahead: r(t+T) is extrapolated from the "
                 "reference's past, which overshoots corners (triangle path size 119 %)."),
        mpc_m2_page(),
        mpc_f_page(),
    ]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text('<mxfile host="Electron" agent="scripts/draw_controller_diagrams.py">'
                   + "".join(p.xml() for p in pages) + "</mxfile>\n")
    print(f"wrote {OUT.relative_to(OUT.parent.parent.parent)} ({len(pages)} pages)")


if __name__ == "__main__":
    main()
