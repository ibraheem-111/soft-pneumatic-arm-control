"""Shared figure style: colour-blind-safe colours, one fixed identity per series, readable sizes.

Every figure script imports this module, calls `apply()` before plotting and draws each series
with `line(name, n)` (keyword arguments for `Axes.plot`), so that a controller looks the same in
every figure: colour + line style + marker. Rules this encodes:

- Colours are the Okabe-Ito set only (readable with red-green colour blindness); yellow only
  for fills. Two series in one panel never differ by colour alone: each series also has its own
  line style and marker, and pairs that a deuteranope confuses (blue / reddish purple, bluish
  green / orange or vermillion) always differ in line style too.
- The reference is a thick black dashed line; "no prediction" bounds are black dotted.
- Figures are about 12 in wide, at most 2 panels per row, panels at least 4.5 in tall, at
  most 4-5 series per panel, lines >= 2 pt, fonts 16 / 14 / 12 / 12 (title / labels / ticks /
  legend), saved at 150 dpi, legends outside the data (`legend_outside`).
- Heat maps use cividis with the value written in each cell (`annotate_cells`); bar charts
  carry their value as text on each bar.

Usage (in a figure script):
    import plot_style as P
    P.apply()
    fig, ax = plt.subplots(figsize=(P.WIDTH, P.PANEL_H))
    ax.plot(t, ref, **P.REFERENCE)
    ax.plot(t, y, **P.line("Delay-aware SMC, no observer (M2)", len(t)))
    P.legend_outside(ax)
    P.save(fig, path)
Run directly to draw a swatch of every series identity:
    uv run python scripts/plot_style.py [out.png]
"""

import sys

import matplotlib
import numpy as np

BLACK, ORANGE, SKY, GREEN = "#000000", "#E69F00", "#56B4E9", "#009E73"
BLUE, VERMILLION, PURPLE, YELLOW = "#0072B2", "#D55E00", "#CC79A7", "#F0E442"
OKABE_ITO = (BLACK, ORANGE, SKY, GREEN, BLUE, VERMILLION, PURPLE)   # YELLOW: fills only

SOLID, DASHED, DASHDOT, DOTTED, LONGDASH = "-", (0, (5, 2.5)), (0, (6, 2, 1.5, 2)), (0, (1.2, 1.8)), (0, (11, 3))

WIDTH = 12.0          # in
PANEL_H = 4.6         # in, per panel row
DPI = 150
LW = 2.2
MARKERS_PER_LINE = 12

# name -> (colour, linestyle, marker, short label). Names are the controller labels used by
# compare_controllers.py, benchmark_slide_tasks.py, sweep_preview_gain.py, benchmark_unified.py.
SERIES = {
    "PD, notebook allocator": (SKY, DOTTED, "v", "PD (as written)"),
    "PD, corrected allocator": (SKY, SOLID, "o", "PD corrected"),
    "SMC (notebook model), as written": (GREEN, DOTTED, "P", "Notebook SMC (as written)"),
    "SMC (notebook model), corrected allocator + deviation, λ=8": (GREEN, SOLID, "s", "Notebook SMC λ=8"),
    "SMC lam=5 K=0.35 phi=0.02, deviation (mpc_controller baseline)":
        (GREEN, DASHDOT, "D", "Notebook SMC λ=5 K=0.35"),
    "Paper SMC, eq. 5 (M1)": (ORANGE, DASHED, "^", "Paper SMC"),
    "Paper NDOSMC, eq. 12 (M1)": (ORANGE, DASHDOT, "v", "Paper NDOSMC"),
    "Delay-aware SMC, no observer (M2)": (BLUE, SOLID, "o", "Delay-aware SMC (Taylor preview)"),
    "Delay-aware NDOSMC (M2)": (BLUE, DASHDOT, "X", "Delay-aware NDOSMC (Taylor preview)"),
    "Delay-aware SMC, RefPredictor r(t+T) (M2)": (PURPLE, LONGDASH, "D", "Delay-aware SMC + RefPredictor"),
    "Delay-aware SMC, exact future reference (M2)": (PURPLE, DOTTED, "*", "Delay-aware SMC, exact r(t+T) [diag]"),
    "MPC, predicted reference": (VERMILLION, SOLID, "s", "MPC (predicted reference)"),
    "MPC, true future reference [diagnostic]": (VERMILLION, DOTTED, "P", "MPC, true future ref [diag]"),
    # segments (command plots)
    "S2": (ORANGE, SOLID, "o", "S2"),
    "S3": (SKY, DASHED, "s", "S3"),
    "S4": (PURPLE, DASHDOT, "^", "S4"),
}
ALIASES = {"Delay-aware SMC, exact future reference (M2) [diagnostic]":
           "Delay-aware SMC, exact future reference (M2)"}

REFERENCE = dict(color=BLACK, ls=(0, (6, 3)), lw=3.0, label="reference")
NO_PREDICTION = dict(color=BLACK, ls=DOTTED, lw=LW)


def apply() -> None:
    """Set the shared rcParams (fonts, line widths, dpi, Okabe-Ito colour cycle)."""
    matplotlib.rcParams.update({
        "figure.dpi": DPI, "savefig.dpi": DPI, "savefig.bbox": "tight",
        "font.size": 12, "axes.titlesize": 16, "axes.labelsize": 14, "xtick.labelsize": 12,
        "ytick.labelsize": 12, "legend.fontsize": 12, "figure.titlesize": 16,
        "lines.linewidth": LW, "lines.markersize": 7, "axes.grid": True, "grid.alpha": 0.3,
        "axes.prop_cycle": matplotlib.cycler(color=list(OKABE_ITO[1:]) + [BLACK]),
        "image.cmap": "cividis",
    })


def identity(name: str) -> tuple[str, object, str, str]:
    """(colour, linestyle, marker, short label) of a named series."""
    return SERIES[ALIASES.get(name, name)]


def label(name: str) -> str:
    return identity(name)[3]


def line(name: str, n: int | None = None, **kw) -> dict:
    """Keyword arguments for Axes.plot: the series' colour, line style, marker every ~n/12 points."""
    col, ls, mk, lab = identity(name)
    out = dict(color=col, ls=ls, marker=mk, lw=LW, label=lab, markersize=7,
               markevery=max(1, (n or MARKERS_PER_LINE) // MARKERS_PER_LINE))
    out.update(kw)
    return out


def figure(rows: int, cols: int = 1, **kw):
    """Figure of rows x cols panels at the shared size (cols <= 2)."""
    import matplotlib.pyplot as plt
    assert cols <= 2, "at most 2 panels per row"
    return plt.subplots(rows, cols, figsize=(WIDTH, PANEL_H * rows), squeeze=False, **kw)


def legend_outside(ax, **kw) -> None:
    """Legend to the right of the axes, outside the data."""
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0, **kw)


def legend_below(ax, ncol: int = 2, **kw) -> None:
    """Legend under the axes, outside the data."""
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=ncol, **kw)


def annotate_cells(ax, values: np.ndarray, image, fmt, shown: np.ndarray | None = None) -> None:
    """Write fmt(shown) (default: the coloured values) in each heat-map cell, black or white for
    contrast with the cell colour."""
    shown = values if shown is None else shown
    for i in range(values.shape[0]):
        for j in range(values.shape[1]):
            rgb = np.asarray(image.cmap(image.norm(values[i, j]))[:3])
            dark = float(rgb @ np.array([0.299, 0.587, 0.114])) < 0.5
            ax.text(j, i, fmt(shown[i, j]), ha="center", va="center", fontsize=11,
                    color="white" if dark else "black")


def save(fig, path) -> None:
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    import matplotlib.pyplot as plt
    plt.close(fig)


def main() -> None:
    matplotlib.use("Agg")
    apply()
    names = [k for k in SERIES if k not in ("S2", "S3", "S4")] + ["S2", "S3", "S4"]
    fig, axs = figure(1, 1)
    ax = axs[0, 0]
    fig.set_size_inches(WIDTH, 0.45 * len(names) + 1)
    x = np.linspace(0, 1, 60)
    for i, nm in enumerate(names):
        ax.plot(x, np.full_like(x, -i), **line(nm, len(x)))
        ax.text(1.03, -i, label(nm), va="center", fontsize=12)
    ax.plot(x, np.full_like(x, -len(names)), **REFERENCE)
    ax.text(1.03, -len(names), "reference", va="center", fontsize=12)
    ax.set(xlim=(0, 1.6), yticks=[], xticks=[], title="Series identities (plot_style.SERIES)")
    ax.grid(False)
    save(fig, sys.argv[1] if len(sys.argv) > 1 else "plot_style_swatch.png")


if __name__ == "__main__":
    main()
