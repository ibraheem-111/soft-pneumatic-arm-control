"""Render the scored circular-tracking run of the best controllers as one side-by-side video.

Re-runs the package harness's loop exactly (reset, 5 s settle at the 4.5 psi baseline,
home = settled tip, then 30 s of `make_reference` tracking with commands clipped to
0-9 psi) on `make_calibrated_sim(seed=0)`, rendering frames as it goes, and checks the
resulting RMSE against `evaluate_controller`. Each tile shows MuJoCo's off-screen view
next to a top-down trace of the tip (x, y about home) over the reference circle, since
7 mm is hard to see in 3-D.

Videos are written to videos/ (git-ignored: *.mp4). Needs an off-screen GL backend;
MUJOCO_GL=egl is set unless already set.

Usage:
    uv run python scripts/render_runs.py              # 2x2 grid of the four best controllers
    uv run python scripts/render_runs.py --each       # also one video per controller
"""

import argparse
import os

os.environ.setdefault("MUJOCO_GL", "egl")

from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import compare_controllers as C
from soft_robotic_arm import make_calibrated_sim
from soft_robotic_arm.evaluation import CircularTrackingTask, make_reference

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "videos"
FPS = 20
TILE_H = 300
SCENE_W = 400
TRACE_W = 300
SPAN_MM = 9.0  # half-width of the trace panel
PICK = [
    "PD, corrected allocator",
    "SMC (notebook model), corrected allocator + deviation, λ=8",
    "Paper SMC, eq. 5 (M1)",
    "Delay-aware NDOSMC (M2)",
]
COLORS = {PICK[0]: (120, 125, 130), PICK[1]: (31, 111, 122), PICK[2]: (154, 106, 18), PICK[3]: (106, 76, 156)}


def font(size: int):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow < 10.1
        return ImageFont.load_default()


def run_and_render(name: str, make, task: CircularTrackingTask, seed: int = 0):
    """Harness loop with rendering; returns frames (list of tiles) and the run's RMSE."""
    sim = make_calibrated_sim(control_hz=task.control_hz, seed=seed)
    stride = int(round(task.control_hz / FPS))
    limit = min(float(task.command_limit_psi), float(sim.cfg.p_max))
    baseline = task.baseline_command(limit)
    ctrl = make()
    obs = sim.reset()
    frames, trail, errs = [], [], []
    for k in range(task.settle_steps):
        obs = sim.step(baseline)
        if k % stride == 0:
            frames.append((sim.render_frame(width=SCENE_W, height=TILE_H), None, None, -task.settle_s + k / task.control_hz))
    home = np.asarray(obs["tip_pos"], float).copy()
    for k in range(task.track_steps):
        t = k / task.control_hz
        ref = make_reference(home, t, task)
        cmd = np.clip(np.asarray(ctrl.compute(float(obs["time"]), obs, ref), float), 0.0, limit)
        obs = sim.step(cmd)
        tip = np.asarray(obs["tip_pos"], float)
        errs.append(np.linalg.norm(tip[:2] - ref[:2]) * 1000.0)
        trail.append(((tip - home) * 1000.0)[:2])
        if k % stride == 0:
            frames.append((sim.render_frame(width=SCENE_W, height=TILE_H), np.array(trail), errs[-1], t))
    sim.close()
    rmse = float(np.sqrt(np.mean(np.square(errs))))
    radius_mm = task.radius_m * 1000.0
    return [tile(name, f, radius_mm, rmse) for f in frames], rmse


def tile(name: str, frame, radius_mm: float, rmse: float) -> np.ndarray:
    scene, trail, err, t = frame
    img = Image.new("RGB", (SCENE_W + TRACE_W, TILE_H + 34), (250, 250, 248))
    img.paste(Image.fromarray(scene), (0, 34))
    d = ImageDraw.Draw(img)
    d.text((8, 6), name.replace("λ", "lambda "), fill=COLORS[name], font=font(15))  # default font has no λ
    # trace panel
    ox, oy, s = SCENE_W + TRACE_W / 2, 34 + TILE_H / 2, (TRACE_W / 2 - 14) / SPAN_MM
    d.rectangle([SCENE_W, 34, SCENE_W + TRACE_W - 1, 34 + TILE_H - 1], fill=(255, 255, 255), outline=(215, 218, 216))
    d.line([SCENE_W + 8, oy, SCENE_W + TRACE_W - 8, oy], fill=(232, 233, 232))
    d.line([ox, 42, ox, 34 + TILE_H - 8], fill=(232, 233, 232))
    r = radius_mm * s
    d.ellipse([ox - r, oy - r, ox + r, oy + r], outline=(40, 40, 40), width=1)
    if trail is not None and len(trail) > 1:
        pts = [(ox + p[0] * s, oy - p[1] * s) for p in trail]
        d.line(pts, fill=COLORS[name], width=2)
        d.ellipse([pts[-1][0] - 4, pts[-1][1] - 4, pts[-1][0] + 4, pts[-1][1] + 4], fill=COLORS[name])
    label = "settling" if trail is None else f"t = {t:4.1f} s   error {err:4.2f} mm"
    d.text((SCENE_W + 8, 40), label, fill=(60, 66, 70), font=font(13))
    d.text((SCENE_W + 8, 34 + TILE_H - 22), f"RMSE (30 s) {rmse:.2f} mm   ±{SPAN_MM:.0f} mm", fill=(60, 66, 70), font=font(13))
    return np.asarray(img)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--each", action="store_true", help="also write one video per controller")
    args = parser.parse_args()
    OUT.mkdir(exist_ok=True)
    task = CircularTrackingTask()
    makers = {name: make for name, _, make in C.controllers()}
    scored = {}
    tiles = {}
    for name in PICK:
        tiles[name], rmse = run_and_render(name, makers[name], task)
        harness = C.run(makers[name], task, 0).rmse_mm
        scored[name] = (rmse, harness)
        print(f"{name:<60} rendered-run RMSE {rmse:.3f} mm, harness {harness:.3f} mm")
        if args.each:
            slug = name.split(",")[0].lower().replace(" ", "_").replace("(", "").replace(")", "")
            imageio.mimsave(OUT / f"run_{PICK.index(name)}_{slug}.mp4", tiles[name], fps=FPS, quality=7)
    n = min(len(v) for v in tiles.values())
    grid = [np.vstack([np.hstack([tiles[PICK[0]][i], tiles[PICK[1]][i]]),
                       np.hstack([tiles[PICK[2]][i], tiles[PICK[3]][i]])]) for i in range(n)]
    path = OUT / "best_controllers_scored_task.mp4"
    imageio.mimsave(path, grid, fps=FPS, quality=7, macro_block_size=8)
    print(f"wrote {path} ({n} frames at {FPS} fps)")


if __name__ == "__main__":
    main()
