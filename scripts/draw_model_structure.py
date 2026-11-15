"""Draw the model-structure figure for the report: how the arm is split into an actuator model and a tip model.

Left: the arm's cross-section with the four pressure columns S1-S4 and the direction each one pushes the tip.
Right: the signal chain command -> transport delay -> pneumatic lag -> pouch pressure -> pressure-to-tip map ->
arm mechanics -> tip position, with the two measured signals (pressure sensors, tip tracking) that split the
identification into two independent fits. Black and white, serif text, no equations (they are in the report text).

Writes docs/figures/model_structure.png (7 x 2 in, 300 dpi).

Usage:
    uv run python scripts/draw_model_structure.py
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle  # noqa: E402

from softarm.constants import FIG  # noqa: E402

FONT = {"family": "serif", "serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"]}
LW = 1.0


def arrow(ax, x0, y0, x1, y1, **kw):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>", mutation_scale=9, lw=LW, color="black",
                                 shrinkA=0, shrinkB=0, **kw))


def block(ax, x, y, w, h, text):
    ax.add_patch(Rectangle((x, y), w, h, fill=False, lw=LW))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=8.5, linespacing=1.15)


def bracket(ax, x0, x1, y, text):
    ax.plot([x0, x0, x1, x1], [y + 0.06, y, y, y + 0.06], color="black", lw=0.8)
    ax.text((x0 + x1) / 2, y - 0.07, text, ha="center", va="top", fontsize=8.5, style="italic")


def cross_section(ax, cx, cy, r):
    ax.add_patch(Circle((cx, cy), r, fill=False, lw=LW))
    pos = {"S1": (1, 0), "S2": (0, 1), "S3": (-1, 0), "S4": (0, -1)}
    push = {"S1": "+x", "S2": "+y", "S3": "−x", "S4": "−y"}
    for name, (dx, dy) in pos.items():
        px, py = cx + 0.62 * r * dx, cy + 0.62 * r * dy
        ax.add_patch(Circle((px, py), 0.2 * r, fill=name == "S1", facecolor="0.75", lw=LW))
        lx, ly = cx + 1.75 * r * dx, cy + 1.3 * r * dy
        ax.text(lx, ly, f"{name} ({push[name]})" if dy else f"{name}\n({push[name]})", ha="center",
                va="center", fontsize=7.5, linespacing=1.0)
    ax.text(cx, cy - 1.75 * r, "Cross-section: four pressure columns\n(S1 sealed on make_calibrated_sim)",
            ha="center", va="top", fontsize=7.5, linespacing=1.1)


def main() -> None:
    plt.rc("font", **FONT)
    fig, ax = plt.subplots(figsize=(7.0, 2.0))
    ax.set_xlim(0, 7.0)
    ax.set_ylim(0, 2.0)
    ax.set_axis_off()

    cross_section(ax, 0.62, 1.12, 0.33)

    yb, hb, wb = 1.02, 0.42, 0.95
    xs = [2.0, 3.12, 4.55, 5.66]
    labels = ["Transport\ndelay T", "Pneumatic lag\nτ, gain G", "Pressure-to-\ntip map H", "Arm\nmechanics a"]
    for x, lab in zip(xs, labels):
        block(ax, x, yb, wb, hb, lab)
    ym = yb + hb / 2
    ax.text(1.72, ym + 0.06, "u", ha="center", va="bottom", fontsize=8.5, style="italic")
    arrow(ax, 1.47, ym, xs[0], ym)
    arrow(ax, xs[0] + wb, ym, xs[1], ym)
    # pouch pressure, measured
    xp = (xs[1] + wb + xs[2]) / 2
    arrow(ax, xs[1] + wb, ym, xs[2], ym)
    ax.plot(xp, ym, "o", color="black", ms=3)
    arrow(ax, xp, ym, xp, ym + 0.42)
    ax.text(xp, ym + 0.47, "pouch pressure p\n(pressure sensors)", ha="center", va="bottom", fontsize=7.5,
            linespacing=1.1)
    arrow(ax, xs[2] + wb, ym, xs[3], ym)
    # tip position, measured
    xt = xs[3] + wb + 0.17
    arrow(ax, xs[3] + wb, ym, 6.98, ym)
    ax.plot(xt, ym, "o", color="black", ms=3)
    arrow(ax, xt, ym, xt, ym + 0.42)
    ax.text(xt, ym + 0.47, "tip position y\n(tip tracking)", ha="center", va="bottom", fontsize=7.5,
            linespacing=1.1)

    bracket(ax, xs[0], xs[1] + wb, yb - 0.12, "Actuator model, per segment: u → p")
    bracket(ax, xs[2], xs[3] + wb, yb - 0.12, "Tip model, per axis: p → y")
    ax.text(xs[0] + wb / 2, yb + hb + 0.06, "T = 0.5 s on make_calibrated_sim\nT = 0 on make_sim", ha="center",
            va="bottom", fontsize=7, linespacing=1.1)

    out = FIG / "model_structure.png"
    fig.savefig(out, dpi=300, bbox_inches="tight", facecolor="white")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
