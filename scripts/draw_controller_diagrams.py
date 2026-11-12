"""Write draw.io block diagrams of the controllers in compare_m2_controllers.py.

One page per controller, drawn from the code:
  1. MPC (M2), submission: the controller of notebooks/Soft_robotic_arm_MPC_M2sim.ipynb (make_sim,
     trajectory preview), in IEEE style (black lines, Times New Roman, no colour) for the report
  2. Delay-aware SMC, reference known (lambda = 8 and the lambda = 4 row; observer off)
  3. Delay-aware NDOSMC, reference known (observer on)
  4. MPC on M2, reference known (make_calibrated_sim)
  5. MPC on F, reference known
Pages 2-5 (softarm.controllers.DelayAwareNDOSMC, DelayMPC, KalmanDelayMPC): blue blocks run in the
controller every 10 ms, dashed blocks are models the controller runs internally, grey is the simulator.
Signals are in mm (tip) and psi (pressure).

Hand edits made in draw.io are overwritten when this script is run again.

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


# --------------------------------------------------------------- IEEE-style page (report figure)

FONT = "fontFamily=Times New Roman;fontColor=#000000;"
I_BLOCK = "rounded=0;whiteSpace=wrap;html=1;fillColor=#FFFFFF;strokeColor=#000000;strokeWidth=1;fontSize=14;" + FONT
I_TEXT = "text;html=1;align=center;verticalAlign=middle;fontSize=14;" + FONT
I_NOTE = "text;html=1;align=left;verticalAlign=top;whiteSpace=wrap;fontSize=12;" + FONT
I_DOT = "ellipse;html=1;fillColor=#000000;strokeColor=#000000;"
I_GROUP = ("rounded=0;html=1;fillColor=none;dashed=1;dashPattern=6 4;strokeColor=#000000;strokeWidth=1;"
           "verticalAlign=top;align=left;spacingLeft=6;spacingTop=2;fontSize=12;" + FONT)
I_EDGE = ("edgeStyle=orthogonalEdgeStyle;rounded=0;html=1;endArrow=classic;endFill=1;endSize=7;strokeColor=#000000;"
          "strokeWidth=1;fontSize=13;labelBackgroundColor=#FFFFFF;" + FONT)


class IEEEPage(Page):
    """Black on white, Times New Roman, no title (the caption goes in the report)."""

    def __init__(self, name: str, width: int, height: int):
        self.name, self.cells, self.n, self.size = name, [], 0, (width, height)

    def dot(self, key: str, cx, cy, r=3) -> None:
        self.box(key, "", cx - r, cy - r, 2 * r, 2 * r, I_DOT)

    def link(self, a: str, b: str, label: str = "", out="r", inn="l", pts=(), lx=0.0, ly=0.0, arrow=True) -> None:
        ox, oy = PORT[out] if isinstance(out, str) else out
        ix, iy = PORT[inn] if isinstance(inn, str) else inn
        s = I_EDGE + f"exitX={ox};exitY={oy};exitDx=0;exitDy=0;entryX={ix};entryY={iy};entryDx=0;entryDy=0;"
        if not arrow:
            s += "endArrow=none;"
        geo = '<mxGeometry relative="1" as="geometry"' + (f' x="{lx}" y="{ly}">' if (lx or ly) else ">")
        if pts:
            geo += '<Array as="points">' + "".join(f'<mxPoint x="{px}" y="{py}"/>' for px, py in pts) + "</Array>"
        self.cells.append(f'<mxCell id="{self._id()}" value="{escape(label)}" style="{s}" edge="1" parent="1" '
                          f'source="{a}" target="{b}">{geo}</mxGeometry></mxCell>')

    def line(self, points, width=1.5, arrow=False) -> None:
        """Free line through points (not attached to blocks); used for glyphs."""
        (x0, y0), (x1, y1) = points[0], points[-1]
        mids = "".join(f'<mxPoint x="{px}" y="{py}"/>' for px, py in points[1:-1])
        end = "endArrow=classic;endSize=3;" if arrow else "endArrow=none;"
        self.cells.append(f'<mxCell id="{self._id()}" style="{end}html=1;strokeColor=#000000;rounded=0;'
                          f'strokeWidth={width};" edge="1" parent="1"><mxGeometry relative="1" as="geometry">'
                          f'<mxPoint x="{x0}" y="{y0}" as="sourcePoint"/><mxPoint x="{x1}" y="{y1}" as="targetPoint"/>'
                          + (f'<Array as="points">{mids}</Array>' if mids else "") + "</mxGeometry></mxCell>")

    def xml(self) -> str:
        w, h = self.size
        return (f'<diagram name="{escape(self.name)}" id="{escape(self.name.replace(" ", "_"))}">'
                f'<mxGraphModel dx="{w}" dy="{h}" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" '
                f'arrows="1" fold="1" page="1" pageScale="1" pageWidth="{w}" pageHeight="{h}" math="0" shadow="0" '
                f'background="#FFFFFF"><root><mxCell id="0"/><mxCell id="1" parent="0"/>{"".join(self.cells)}'
                '</root></mxGraphModel></diagram>')


def submission_page() -> IEEEPage:
    """The submitted controller: MPC with trajectory preview on M2 identified on make_sim (controller cell of
    notebooks/Soft_robotic_arm_MPC_M2sim.ipynb). No dead time on this plant; four segments in opposing pairs."""
    p = IEEEPage("MPC M2 submission (IEEE)", 1450, 470)
    i = lambda s: f"<i>{s}</i>"          # noqa: E731
    sub = lambda s: f"<sub>{s}</sub>"    # noqa: E731
    db = "<i>δ̄</i>"                       # applied correction
    y0, y1 = 70, 270                     # forward path, feedback row

    p.box("ref", f"{i('r')}({i('t')})", 0, y0 - 15, 40, 30, I_TEXT)
    p.box("prev", f"Reference preview<br>{i('R')}{sub(i('k'))} = [ {i('r')}({i('t')}{sub(i('k'))} + {i('i')}Δ{i('t')}) ], "
                  f"{i('i')} = 1 … {i('H')}", 60, y0 - 32, 190, 64, I_BLOCK)
    p.box("mpc", f"Unconstrained MPC (first move)<br>{i('δ')}{sub(i('k'))} = {i('g')}{sub(i('r'))}{i('R')}{sub(i('k'))} + "
                 f"{i('g')}{sub(i('x'))}{i('X')}{sub(i('k'))} + {i('g')}{sub(i('w'))}{i('w')}{sub(i('k'))} + "
                 f"{i('g')}{sub(i('u'))}{db}{sub(i('k') + '−1')}", 300, y0 - 38, 270, 76, I_BLOCK)
    p.box("alloc", f"Pair allocation<br>4.5 + {i('P')}{i('δ')}{sub(i('k'))}", 610, y0 - 32, 120, 64, I_BLOCK)
    p.box("sat", "<br><br>[0, 9] psi", 770, y0 - 32, 80, 64, I_BLOCK)
    cx, cy = 810, y0 - 8                                     # saturation glyph
    p.line([(cx - 22, cy), (cx + 22, cy)], width=0.6, arrow=True)
    p.line([(cx, cy + 14), (cx, cy - 16)], width=0.6, arrow=True)
    p.line([(cx - 20, cy + 10), (cx - 9, cy + 10), (cx + 9, cy - 10), (cx + 20, cy - 10)])
    p.box("act", f"{i('G')} / ({i('τs')} + 1)", 930, y0 - 25, 105, 50, I_BLOCK)
    p.box("H", i("H"), 1090, y0 - 25, 50, 50, I_BLOCK)
    p.box("mech", f"{i('a')}{sub(i('i'))} / ({i('s')} + {i('a')}{sub(i('i'))})", 1195, y0 - 25, 115, 50, I_BLOCK)
    p.box("y", i("y"), 1395, y0 - 15, 30, 30, I_TEXT)
    p.box("plant", "Plant (<span style=\"font-family:Courier New\">make_sim</span>) as modelled (M2), "
                   f"axes {i('i')} ∈ {{{i('x')}, {i('y')}}}", 910, y0 - 68, 420, 120, I_GROUP)
    p.dot("utap", 885, y0)
    p.dot("ptap", 1062, y0)
    p.dot("ytap", 1350, y0)
    p.link("ref", "prev")
    p.link("prev", "mpc", f"{i('R')}{sub(i('k'))}", ly=-12)
    p.link("mpc", "alloc", f"{i('δ')}{sub(i('k'))}", ly=-12)
    p.link("alloc", "sat")
    p.link("sat", "utap", f"{i('u')}{sub(i('k'))}", ly=-12, arrow=False)
    p.link("utap", "act")
    p.link("act", "ptap", i("p"), ly=-12, arrow=False)
    p.link("ptap", "H")
    p.link("H", "mech", i("v"), ly=-12)
    p.link("mech", "ytap", arrow=False)
    p.link("ytap", "y")

    p.box("state", f"State<br>{i('X')}{sub(i('k'))} = ( {i('y')}{sub(i('k'))}, {i('p')}{sub(i('k'))} − 4.5 )",
          90, y1 - 30, 190, 60, I_BLOCK)
    p.box("offset", f"Offset estimator<br>{i('e')}{sub(i('k'))} = {i('y')}{sub(i('k'))} − {i('C')}({i('A')}{i('X')}"
                    f"{sub(i('k') + '−1')} + {i('B')}{db}{sub(i('k') + '−1')})<br>{i('w')}{sub(i('k'))} = "
                    f"{i('w')}{sub(i('k') + '−1')} + {i('α')}({i('e')}{sub(i('k'))} − {i('w')}{sub(i('k') + '−1')})",
          360, y1 - 42, 300, 84, I_BLOCK)
    p.box("z1", f"{i('z')}<sup>−1</sup>", 740, y1 - 22, 50, 44, I_BLOCK)
    p.box("rec", f"Applied correction<br>{db}{sub(i('k'))} = ½ ( {i('u')}{sub('1')} − {i('u')}{sub('3')}, "
                 f"{i('u')}{sub('2')} − {i('u')}{sub('4')} )", 830, y1 - 30, 200, 60, I_BLOCK)
    p.dot("dtap", 700, y1)
    yp, yy = 360, 385                                        # measurement buses
    p.link("ytap", "state", f"{i('y')}{sub(i('k'))} (tip position)", out="b", inn=(0.3, 1),
           pts=[(1350, yy), (147, yy)], lx=0.55, ly=-12)
    p.link("ptap", "state", f"{i('p')}{sub(i('k'))} (segment pressure sensors)", out="b", inn=(0.7, 1),
           pts=[(1062, yp), (223, yp)], lx=0.25, ly=-12)
    p.link("utap", "rec", out="b", inn=(0.275, 0), pts=[(885, y1 - 30)])
    p.link("rec", "z1", f"{db}{sub(i('k'))}", out="l", inn="r", ly=-12)
    p.link("z1", "dtap", out="l", inn="r", arrow=False)
    p.link("dtap", "offset", f"{db}{sub(i('k') + '−1')}", out="l", inn="r", ly=14)
    p.link("state", "offset", f"{i('X')}{sub(i('k'))}", out="r", inn="l", ly=-12)
    p.link("state", "mpc", f"{i('X')}{sub(i('k'))}", out="t", inn=(0.15, 1), pts=[(185, 180), (340, 180)], lx=0.85)
    p.link("offset", "mpc", f"{i('w')}{sub(i('k'))}", out="t", inn=(0.55, 1), pts=[(510, 195), (448, 195)], lx=0.6)
    p.link("dtap", "mpc", f"{db}{sub(i('k') + '−1')}", out="t", inn=(0.9, 1), pts=[(700, 165), (543, 165)], lx=0.3)
    p.box("note", f"Δ{i('t')} = 10 ms; horizon {i('H')} = 100 steps (1 s); {i('R')}{sub(i('u'))} = 0.1 mm²/psi²; "
                  f"{i('τ')} = 0.60 s; {i('G')} = 1; {i('a')}{sub(i('x'))} = 4.66 s⁻¹, {i('a')}{sub(i('y'))} = 4.69 s⁻¹; "
                  f"{i('H')} ∈ ℝ<sup>2×4</sup> (≈ 1.84 mm/psi); {i('P')} = [1 0 −1 0; 0 1 0 −1]<sup>T</sup> maps "
                  f"({i('δ')}{sub(i('x'))}, {i('δ')}{sub(i('y'))}) to S1–S4; {i('α')} = 1 − e<sup>−Δ<i>t</i>/0.5 s</sup>; "
                  "each segment command drives its five pouches.", 90, 400, 1240, 40, I_NOTE)
    return p


LEGEND = ("Blue: computed by the controller every 10 ms. Dashed: model run inside the controller. "
          "Grey: the simulator (make_calibrated_sim). Tip q in mm from the settled pose; pressures in psi.")
PLANT_LABEL = ("<b>Plant</b>: make_calibrated_sim<br>dead time T = 0.5 s<br>→ pneumatic lag τ ≈ 0.6 s<br>"
               "→ pouches → MuJoCo arm")


def smc_page(name: str, title: str, observer: bool, note: str) -> Page:
    """Delay-aware (ND)OSMC: look-ahead on the known reference, Smith predictor on M2, lag inverse."""
    p = Page(name, title)
    p.box("grp", "Controller (every 10 ms, per axis x, y, z)", 110, 70, 1300, 560, GROUP)
    p.box("ref", "Known trajectory<br>r(t), all t", 150, 95, 150, 60, TEXT)
    p.box("look", "<b>Look-ahead</b><br>r(t+T), ṙ(t+T) read from the trajectory<br>T = 0.5 s (N = 50 steps)",
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
    p.box("note", "Code: softarm.controllers.DelayMPC on softarm.models.M2. M2: τ ṗ = −p + G u(t−T), ẏ = −Λ y + Λ H p "
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
    p.box("note", "Code: softarm.controllers.KalmanDelayMPC(preview=\"oracle\") with the true "
          "future reference, as built by softarm.controllers.known_reference. F: per-channel state-space model "
          "x<sub>n+1</sub> = A<sub>d</sub> x<sub>n</sub> + B<sub>d</sub> u<sub>n−d</sub> identified from "
          "frequency responses (scripts/sysid_frf.py). Unlike M2, F does not model z and treats x and y "
          "as independent channels.<br><br>" + LEGEND, 60, 650, 1250, 110, NOTE)
    return p


def main() -> None:
    pages = [
        submission_page(),
        smc_page("SMC known ref", "Delay-aware SMC (M2), reference known",
                 False,
                 "Code: softarm.controllers.DelayAwareNDOSMC(observer=False, exact_preview=True), fed r(t+T) by "
                 "softarm.controllers.Ahead. Rows: λ = 8 s⁻¹ and λ = 4 s⁻¹; μ = 1 mm, ε = 1. "
                 "a, b, H, G, τ from M2 (docs/figures/task_space_model.json); the observer is off (d̂ = 0)."),
        smc_page("NDOSMC known ref", "Delay-aware NDOSMC (M2), reference known",
                 True,
                 "Code: softarm.controllers.DelayAwareNDOSMC(exact_preview=True) with the observer on. "
                 "Nikoletta's gains: λ = 4 s⁻¹, l₁ = 0.1, l₂ = 0, μ = 1 mm, Φ = 1, ε = 1. Same structure as the "
                 "SMC page plus the observer, which acts on the predicted tip q̂."),
        mpc_m2_page(),
        mpc_f_page(),
    ]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text('<mxfile host="Electron" agent="scripts/draw_controller_diagrams.py">'
                   + "".join(p.xml() for p in pages) + "</mxfile>\n")
    print(f"wrote {OUT.relative_to(OUT.parent.parent.parent)} ({len(pages)} pages)")


if __name__ == "__main__":
    main()
