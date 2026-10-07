"""Draw the kinematic diagram of the coursework soft arm.

Every dimension and joint is read from the packaged MuJoCo model, so the figure
stays consistent with the simulator. Produces three panels:

  (a) kinematic schematic in the rest pose (x-z side view)
  (b) the same chain in an illustrative bent pose, computed by MuJoCo FK
  (c) top-view cross-section showing the four pouch columns

Usage:
    uv run python scripts/draw_kinematics.py            # writes docs/figures/kinematic_diagram.{svg,png,pdf}
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from soft_robotic_arm import make_sim

OUT_DIR = Path(__file__).resolve().parent.parent / "docs" / "figures"

INK = "#1f2328"
MUTED = "#6e7781"
LINK = "#24292f"
PRISMATIC = "#0969da"
HINGE_X = "#bf3989"
HINGE_Y = "#1a7f37"
# Column colors match the MuJoCo model (S1 orange, S2 green, S3 red, S4 blue).
COLUMN_COLORS = ("#eb8c14", "#1aad2e", "#d12e14", "#1447e6")
COLUMN_LABELS = ("S1", "S2", "S3", "S4")
# Tip tendency of each column, verified against the simulator in main().
COLUMN_TENDENCY = ("+x", "+y", "−x", "−y")

N_LEVELS = 5


def read_geometry(sim):
    """Pull every dimension the drawing needs from the compiled model."""
    m, d = sim.model, sim.data
    mujoco.mj_forward(m, d)
    cfg = sim.cfg
    mm = 1000.0
    return {
        "level_h": cfg.length / cfg.n_pouches * mm,
        "tip_disc": cfg.length / cfg.n_pouches * mm,   # tip_disc sits one level below level4
        "tip_frame": 12.0,                              # tip_frame offset in model.py [mm]
        "mount_z": d.body("mount").xpos[2] * mm,
        "tip_z": d.site("tip").xpos[2] * mm,
        "col_offset": cfg.col_offset * mm,
        "col_radius": cfg.col_radius * mm,
        "tip_arm": cfg.tip_arm * mm,
        "k_bend": cfg.base_stiffness,
        "c_bend": cfg.base_damping,
        "k_ax": cfg.axial_stiffness,
        "c_ax": cfg.axial_damping,
        "ext_range": m.jnt_range[m.joint("ext0").id] * mm,
        "joint_names": [m.joint(j).name for j in range(m.njnt)],
    }


def verify_column_tendency():
    """Confirm each column's tip tendency so the labels cannot drift from the sim."""
    observed = []
    for s in range(4):
        sim = make_sim(seed=0)
        obs = sim.reset()
        home = obs["tip_pos"][:2].copy()
        cmd = np.full((4, 5), 2.0)
        cmd[s] = 6.0
        for _ in range(300):
            obs = sim.step(cmd)
        dx, dy = obs["tip_pos"][:2] - home
        axis = "x" if abs(dx) > abs(dy) else "y"
        sign = "+" if (dx if axis == "x" else dy) > 0 else "−"
        observed.append(sign + axis)
        sim.close()
    if tuple(observed) != COLUMN_TENDENCY:
        raise RuntimeError(f"column tendencies changed: {observed}")


# ── joint symbols (standard kinematic-diagram conventions) ──────────────────
def draw_ground(ax, x, z, width):
    ax.plot([x - width / 2, x + width / 2], [z, z], color=INK, lw=2.2, solid_capstyle="butt")
    for xi in np.linspace(x - width / 2, x + width / 2 - 4, 11):
        ax.plot([xi, xi + 4], [z, z + 4], color=INK, lw=0.9)


def draw_prismatic(ax, x, z_top, z_bot):
    """Sleeve with a sliding rod: translation along the vertical (−z) axis."""
    w, h = 9.0, (z_top - z_bot) * 0.6
    zc = (z_top + z_bot) / 2
    ax.plot([x, x], [z_top, zc + h / 2], color=LINK, lw=2.4, zorder=2)
    ax.add_patch(mpatches.Rectangle((x - w / 2, zc - h / 2), w, h, fc="white",
                                    ec=PRISMATIC, lw=1.8, zorder=3))
    ax.plot([x, x], [zc + h / 2 - 2, z_bot], color=PRISMATIC, lw=2.6, zorder=4)


def draw_hinge_in_plane(ax, x, z):
    """Revolute joint whose axis lies in the drawing plane (here: x)."""
    w, h = 13.0, 6.0
    ax.add_patch(mpatches.FancyBboxPatch((x - w / 2, z - h / 2), w, h,
                                         boxstyle="round,pad=0,rounding_size=1.2",
                                         fc="white", ec=HINGE_X, lw=1.8, zorder=5))
    for xe in (x - w / 2 + 2.2, x + w / 2 - 2.2):
        ax.plot([xe, xe], [z - h / 2, z + h / 2], color=HINGE_X, lw=1.1, zorder=6)


def draw_hinge_out_of_plane(ax, x, z):
    """Revolute joint whose axis is normal to the drawing plane (here: y)."""
    ax.add_patch(mpatches.Circle((x, z), 4.2, fc="white", ec=HINGE_Y, lw=1.8, zorder=5))
    ax.add_patch(mpatches.Circle((x, z), 1.0, fc=HINGE_Y, ec="none", zorder=6))


def draw_frame(ax, x, z, size=10.0, label=None):
    """Small body frame: x to the right, z up (y into the page)."""
    kw = dict(arrowstyle="-|>", mutation_scale=7, lw=0.9, color=MUTED, zorder=7)
    ax.add_patch(mpatches.FancyArrowPatch((x, z), (x + size, z), **kw))
    ax.add_patch(mpatches.FancyArrowPatch((x, z), (x, z + size), **kw))
    ax.text(x + size + 1, z, "x", fontsize=6.5, color=MUTED, va="center")
    ax.text(x, z + size + 1, "z", fontsize=6.5, color=MUTED, ha="center", va="bottom")
    if label:
        ax.text(x - 2, z - 1.5, label, fontsize=7, color=MUTED, ha="right", va="top")


# ── panels ──────────────────────────────────────────────────────────────────
def panel_schematic(ax, g):
    """(a) Rest-pose schematic. Joints of one level are drawn slightly apart for
    legibility; in the model they all act at the level origin O_k."""
    h = g["level_h"]
    z0 = g["mount_z"]
    draw_ground(ax, 0, z0 + 6, 70)
    ax.plot([0, 0], [z0 + 6, z0], color=LINK, lw=2.4)
    ax.text(38, z0 + 9, "mount (fixed to world)", fontsize=8, color=INK, va="center")

    for k in range(N_LEVELS):
        zk = z0 - k * h                         # level-k origin O_k
        ax.add_patch(mpatches.Rectangle((-36, zk - h), 72, h, fc="#f6f8fa" if k % 2 else "white",
                                        ec="#d0d7de", lw=0.6, zorder=0))
        z_p_bot = zk - 15
        draw_prismatic(ax, 0, zk, z_p_bot)
        draw_hinge_in_plane(ax, 0, zk - 20)
        draw_hinge_out_of_plane(ax, 0, zk - 28)
        ax.plot([0, 0], [zk - 32.2, zk - h], color=LINK, lw=2.4, zorder=2)
        ax.plot(0, zk, marker="o", ms=3, color=INK, zorder=8)
        ax.text(-34, zk - 3, f"$O_{k}$", fontsize=8, color=INK, va="top")
        ax.text(-34, zk - h + 3, f"level{k}\npouches P{k + 1}", fontsize=6.5, color=MUTED, va="bottom")
        # joint labels on the right, one row per joint
        ax.text(9, zk - 9, f"ext{k}  (P, along −z)", fontsize=7, color=PRISMATIC, va="center")
        ax.text(9, zk - 20, f"bx{k}  (R about x)", fontsize=7, color=HINGE_X, va="center")
        ax.text(9, zk - 28, f"by{k}  (R about y)", fontsize=7, color=HINGE_Y, va="center")

    # level-height dimension on the far left
    zk = z0 - 2 * h
    ax.annotate("", xy=(-44, zk), xytext=(-44, zk - h),
                arrowprops=dict(arrowstyle="<->", color=MUTED, lw=0.8))
    ax.text(-46, zk - h / 2, f"{h:.1f} mm", rotation=90, fontsize=7, color=MUTED,
            ha="right", va="center")

    # tip disc and OptiTrack marker frame
    z_disc = z0 - N_LEVELS * h
    ax.plot([-14, 14], [z_disc, z_disc], color=INK, lw=3, solid_capstyle="butt", zorder=3)
    ax.text(18, z_disc, "tip_disc", fontsize=7, color=INK, va="center")
    z_tip = z_disc - g["tip_frame"]
    ax.plot([0, 0], [z_disc, z_tip], color=LINK, lw=2.4)
    a = g["tip_arm"] * 0.45  # marker cross is 70 mm half-span; drawn shortened
    ax.plot([-a, a], [z_tip, z_tip], color=INK, lw=1.6)
    for xm in (-a, 0, a):
        ax.add_patch(mpatches.Circle((xm, z_tip), 2.0 if xm else 2.6, fc="#d0d7de", ec=INK, lw=0.8, zorder=6))
    ax.text(a + 4, z_tip - 1, "tip site\n(tip_pos, tip_quat, tip_vel)", fontsize=7, color=INK, va="center")
    draw_frame(ax, -62, z0 - 40)
    ax.text(-62, z0 - 44, "world", fontsize=7, color=MUTED, va="top")

    ax.text(8, z_tip - 16,
            "Joints of a level are drawn apart for legibility;\n"
            "in the model all three act at $O_k$, applied in order ext → bx → by.",
            fontsize=7, color=MUTED, ha="center", va="top", style="italic")
    ax.set_xlim(-75, 95)
    ax.set_ylim(z_tip - 34, z0 + 16)
    ax.set_aspect("equal")
    ax.set_title("(a) Kinematic schematic, rest pose (x–z view)", fontsize=10, loc="left", color=INK)
    ax.set_ylabel("height above floor [mm]", fontsize=8, color=MUTED)
    ax.tick_params(labelsize=7, colors=MUTED)
    ax.set_xticks([])
    for spine in ("top", "right", "bottom"):
        ax.spines[spine].set_visible(False)


def panel_bent(ax, g, bend_rad=-0.06):
    """(b) Illustrative pose: equal bend at every level, positions from MuJoCo FK."""
    sim = make_sim(seed=0)
    sim.reset()
    m, d = sim.model, sim.data
    for k in range(N_LEVELS):
        d.qpos[m.jnt_qposadr[m.joint(f"by{k}").id]] = bend_rad
    mujoco.mj_forward(m, d)
    pts = [d.body(f"level{k}").xpos.copy() for k in range(N_LEVELS)]
    pts += [d.body("tip_disc").xpos.copy(), d.site("tip").xpos.copy()]
    pts = np.array(pts) * 1000.0
    rest_tip = np.array([0.0, 0.0, g["tip_z"]])
    sim.close()

    mount = pts[0]
    draw_ground(ax, mount[0], mount[2] + 6, 70)
    ax.plot([mount[0], mount[0]], [mount[2] + 6, mount[2]], color=LINK, lw=2.4)
    # rest-pose ghost
    ax.plot([0, 0], [mount[2], rest_tip[2]], color="#d0d7de", lw=2, ls="--", zorder=1)
    ax.plot(pts[:, 0], pts[:, 2], color=LINK, lw=2.4, zorder=2)
    for k in range(N_LEVELS):
        draw_hinge_out_of_plane(ax, pts[k, 0], pts[k, 2])
        ax.text(pts[k, 0] + 6, pts[k, 2], f"$O_{k}$", fontsize=8, color=INK, va="center")
    ax.plot(*pts[-1, [0, 2]], marker="o", ms=6, mfc="#d0d7de", mec=INK, zorder=6)
    ax.text(pts[-1, 0] + 6, pts[-1, 2], "tip", fontsize=8, color=INK, va="center")
    ax.plot(*rest_tip[[0, 2]], marker="o", ms=5, mfc="white", mec=MUTED, zorder=6)
    ax.text(rest_tip[0] + 5, rest_tip[2] - 6, "rest tip", fontsize=7, color=MUTED, va="top")

    ax.text(mount[0] - 72, mount[2] - 205,
            f"by$_k$ = {bend_rad:+.2f} rad at every level\n"
            f"(illustrative, not a pressure\nsolution; FK computed by MuJoCo)\n\n"
            f"Bending compounds down the chain:\nlevel 0 (P1, at the mount) moves the\n"
            f"tip the most, level 4 (P5) the least.",
            fontsize=7, color=MUTED, va="top")
    ax.set_xlim(mount[0] - 75, max(pts[:, 0].max(), 0) + 45)
    ax.set_ylim(rest_tip[2] - 34, mount[2] + 16)
    ax.set_aspect("equal")
    ax.set_title("(b) Bent pose, x–z view", fontsize=10, loc="left", color=INK)
    ax.set_xlabel("x [mm]", fontsize=8, color=MUTED)
    ax.tick_params(labelsize=7, colors=MUTED)
    ax.set_yticks([])
    for spine in ("top", "right", "left"):
        ax.spines[spine].set_visible(False)


def panel_top(ax, g):
    """(c) Cross-section of one level seen from above."""
    r_off, r_col = g["col_offset"], g["col_radius"]
    ring = r_off + r_col + 3
    ax.add_patch(mpatches.Circle((0, 0), ring, fc="#f6f8fa", ec=INK, lw=1.2))
    for s, phi in enumerate(np.deg2rad([0, 90, 180, 270])):
        cx, cy = r_off * np.cos(phi), r_off * np.sin(phi)
        ax.add_patch(mpatches.Circle((cx, cy), r_col, fc=COLUMN_COLORS[s], ec=INK, lw=0.8, alpha=0.85))
        ax.text(cx, cy, COLUMN_LABELS[s], fontsize=9, color="white", ha="center", va="center",
                weight="bold")
        lx, ly = (ring + 15) * np.cos(phi), (ring + 15) * np.sin(phi)
        ax.text(lx, ly, f"{int(np.rad2deg(phi))}°\ntip → {COLUMN_TENDENCY[s]}",
                fontsize=7, color=INK, ha="center", va="center")
    ax.add_patch(mpatches.Circle((0, 0), 5, fc=INK, ec="none"))

    kw = dict(arrowstyle="-|>", mutation_scale=9, lw=1.0, color=MUTED)
    ax.add_patch(mpatches.FancyArrowPatch((-ring - 30, -ring - 22), (-ring - 12, -ring - 22), **kw))
    ax.add_patch(mpatches.FancyArrowPatch((-ring - 30, -ring - 22), (-ring - 30, -ring - 4), **kw))
    ax.text(-ring - 10, -ring - 22, "x", fontsize=8, color=MUTED, va="center")
    ax.text(-ring - 30, -ring - 2, "y", fontsize=8, color=MUTED, ha="center", va="bottom")

    ax.plot([0, 0], [-ring - 26, -ring - 34], color=MUTED, lw=0.6)
    ax.plot([r_off, r_off], [-ring - 26, -ring - 34], color=MUTED, lw=0.6)
    ax.annotate("", xy=(r_off, -ring - 30), xytext=(0, -ring - 30),
                arrowprops=dict(arrowstyle="<->", color=MUTED, lw=0.8))
    ax.text(r_off + 3, -ring - 30, f"{r_off:.0f} mm centre offset", fontsize=7, color=MUTED, va="center")
    ax.text(0, ring + 38,
            f"column radius {r_col:.0f} mm · black disc = central core\n"
            f"pouch k of every column acts on level k (bending + axial force)",
            fontsize=7, color=MUTED, ha="center", va="bottom")
    lim = ring + 45
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim - 6, lim + 12)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title("(c) Column layout, top view", fontsize=10, loc="left", color=INK)


def legend_and_table(fig, g):
    handles = [
        mpatches.Patch(fc="white", ec=PRISMATIC, lw=1.8, label="prismatic ext$_k$ (axial slide)"),
        mpatches.Patch(fc="white", ec=HINGE_X, lw=1.8, label="revolute bx$_k$ (axis x, in plane)"),
        plt.Line2D([], [], marker="o", ls="none", mfc="white", mec=HINGE_Y, mew=1.8, ms=8,
                   label="revolute by$_k$ (axis y, out of plane)"),
    ]
    fig.legend(handles=handles, loc="lower left", bbox_to_anchor=(0.02, 0.01), ncol=3,
               fontsize=8, frameon=False)
    rows = [
        ("joint", "type", "range", "stiffness", "damping"),
        ("ext$_k$", "slide, −z", f"{g['ext_range'][0]:.0f} … +{g['ext_range'][1]:.0f} mm",
         f"{g['k_ax']:.1f} N/m", f"{g['c_ax']:.2f} N·s/m"),
        ("bx$_k$, by$_k$", "hinge, x / y", "unbounded",
         f"{g['k_bend']:.4f} N·m/rad", f"{g['c_bend']:.4f} N·m·s/rad"),
    ]
    table = fig.add_axes([0.60, 0.015, 0.38, 0.09])
    table.axis("off")
    t = table.table(cellText=rows[1:], colLabels=rows[0], loc="center", cellLoc="left")
    t.auto_set_font_size(False)
    t.set_fontsize(7.5)
    t.scale(1, 1.25)
    for (r, _), cell in t.get_celld().items():
        cell.set_edgecolor("#d0d7de")
        if r == 0:
            cell.set_text_props(weight="bold", color=INK)


def main() -> None:
    verify_column_tendency()
    sim = make_sim(seed=0)
    sim.reset()
    g = read_geometry(sim)
    sim.close()
    expected = [f"{j}{k}" for k in range(N_LEVELS) for j in ("ext", "bx", "by")]
    if g["joint_names"] != expected:
        raise RuntimeError(f"unexpected joint layout: {g['joint_names']}")

    fig = plt.figure(figsize=(15, 9.6))
    grid = fig.add_gridspec(1, 3, width_ratios=[1.15, 1.0, 0.95], left=0.05, right=0.98,
                            top=0.90, bottom=0.14, wspace=0.12)
    panel_schematic(fig.add_subplot(grid[0]), g)
    panel_bent(fig.add_subplot(grid[1]), g)
    panel_top(fig.add_subplot(grid[2]), g)
    legend_and_table(fig, g)
    fig.suptitle("Soft pneumatic arm — kinematic structure of the MuJoCo model "
                 "(5 levels × [P, R, R] = 15 DOF)", fontsize=13, color=INK, x=0.05, ha="left")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("svg", "png", "pdf"):
        fig.savefig(OUT_DIR / f"kinematic_diagram.{ext}", dpi=200 if ext == "png" else None)
    print(f"Wrote {OUT_DIR}/kinematic_diagram.{{svg,png,pdf}}")


if __name__ == "__main__":
    main()
