"""Report figure: tip paths of the delay-aware NDOSMC and the MPC on M2 against the reference.

Reads the traces stored by scripts/compare_m2_controllers.py (docs/figures/m2_controllers.json, tip and
reference every 50 ms, make_calibrated_sim, reference known in advance) and draws, for the triangle, the
square and the random waypoints, the path after settling (last lap for the polygons, 12 s after the 5 s
settle for the waypoints) and below it a 1.6 mm zoom on the point where the NDOSMC strays furthest. Sized for an IEEE
two-column page (7 in wide). Colours are blue and orange so the two controllers stay distinct for red-green
colour blindness; each also keeps its own line style and marker.

Writes docs/figures/report_paths_ndosmc_mpc.png.

Usage:
    uv run python scripts/compare_m2_controllers.py      # traces first (already in the repo)
    uv run python scripts/plot_report_paths.py
"""

import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

from softarm import plotting as P  # noqa: E402
from softarm.constants import FIG  # noqa: E402

NDOSMC, MPC = "Delay-aware NDOSMC (M2, known ref)", "MPC (M2), horizon 100 steps = 1.00 s"
STYLE = {NDOSMC: dict(color=P.BLUE, ls=P.DASHED, lw=1.4, label="NDOSMC"),
         MPC: dict(color=P.ORANGE, ls=P.SOLID, lw=1.4, label="MPC")}
REF = dict(color="black", ls=(0, (4, 2)), lw=1.0, label="reference")
TESTS = [("triangle", "Triangle (7 mm, 10 s lap)", 10.0), ("square", "Square (7 mm, 10 s lap)", 10.0),
         ("random waypoints", "Random waypoints", None)]
DT_TRACE = 0.05
ZOOM_MM = 1.6
WAYPOINT_WINDOW_S = 12.0      # waypoints: draw 12 s after settling, so the path stays readable


def traces(runs: list, test: str) -> dict:
    out = {}
    for r in runs:
        if r["test"] == test and r["controller"] in (NDOSMC, MPC):
            out[r["controller"]] = (np.array(r["tip_mm"]), np.array(r["ref_mm"]), r["metrics"])
    return out


def main() -> None:
    d = json.loads((FIG / "m2_controllers.json").read_text())
    P.apply()
    plt.rcParams.update({"font.size": 8, "axes.titlesize": 9, "axes.labelsize": 8, "xtick.labelsize": 7,
                         "ytick.labelsize": 7, "legend.fontsize": 7.5, "font.family": "serif",
                         "font.serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"]})
    fig, axs = plt.subplots(2, 3, figsize=(7.0, 4.6), layout="constrained", height_ratios=[1.25, 1])
    for j, (test, title, lap_s) in enumerate(TESTS):
        ax, zx = axs[0, j], axs[1, j]
        tr = traces(d["runs"], test)
        tip_n, ref, m_n = tr[NDOSMC]
        m_m = tr[MPC][2]
        n = len(ref)
        k0 = n - int(round(lap_s / DT_TRACE)) if lap_s else int(round(m_n["settle_s"] / DT_TRACE))
        k1 = n if lap_s else min(n, k0 + int(round(WAYPOINT_WINDOW_S / DT_TRACE)))
        # zoom on the point where the NDOSMC strays furthest from the reference path
        dist = np.linalg.norm(tip_n[k0:k1, None, :] - ref[None, k0:k1, :], axis=2).min(1)
        cx, cy = tip_n[k0 + int(np.argmax(dist))]
        for a_ in (ax, zx):
            a_.plot(ref[k0:k1, 0], ref[k0:k1, 1], **REF)
            for name in (NDOSMC, MPC):
                tip = tr[name][0]
                a_.plot(tip[k0:k1, 0], tip[k0:k1, 1], **STYLE[name])
            a_.set_aspect("equal", adjustable="datalim" if a_ is ax else "box")
            a_.grid(alpha=0.3)
        ax.add_patch(Rectangle((cx - ZOOM_MM / 2, cy - ZOOM_MM / 2), ZOOM_MM, ZOOM_MM, fill=False, lw=0.8,
                               ec="0.3"))
        zx.set(xlim=(cx - ZOOM_MM / 2, cx + ZOOM_MM / 2), ylim=(cy - ZOOM_MM / 2, cy + ZOOM_MM / 2))
        ax.set_title(title)
        zx.set_title(f"zoom (box above); shape error\nNDOSMC {m_n['shape_mm']:.2f} mm, MPC {m_m['shape_mm']:.2f} mm",
                     fontsize=7.5)
        ax.set_xlabel("x (mm)")
        zx.set_xlabel("x (mm)")
    axs[0, 0].set_ylabel("y (mm)")
    axs[1, 0].set_ylabel("y (mm)")
    fig.legend(*axs[0, 0].get_legend_handles_labels(), loc="outside lower center", ncol=3, frameon=False)
    out = FIG / "report_paths_ndosmc_mpc.png"
    fig.savefig(out, dpi=300, facecolor="white")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
