"""Interactive MuJoCo viewer for the coursework soft arm.

Runs the real ``make_sim()`` plant (pneumatic lag + pressure-to-joint forces)
inside MuJoCo's passive viewer, with a separate slider window holding one slider
per pouch (rows P1 at the mount .. P5 nearest the tip, columns S1 +x, S2 +y,
S3 -x, S4 -y), laid out like the hanging arm. The
top slider of each column sets all five pouches in that segment at once.
Sliders are used instead of keys because MuJoCo's viewer already binds almost
every letter and digit.

Usage:
    uv run python scripts/view_arm.py
    uv run python scripts/view_arm.py --dump-xml soft_arm.xml   # write MJCF and exit
"""

import argparse
import threading
import time
import tkinter as tk

import mujoco
import mujoco.viewer
import numpy as np

from soft_robotic_arm import make_sim
from soft_robotic_arm.calibration import RobotCalibration
from soft_robotic_arm.model import build_arm_xml

CONTROL_HZ = 100.0
MAX_PRESSURE_PSI = 9.0
BASELINE_PSI = 2.0
SEGMENT_LABELS = ("S1 (+x)", "S2 (+y)", "S3 (−x)", "S4 (−y)")


def build_panel(root: tk.Tk, pouch_cmd: np.ndarray) -> tk.StringVar:
    """Create a segment slider plus five pouch sliders per segment.

    Returns the status-line variable. ``pouch_cmd`` (4, 5) is updated in place.
    """
    root.title("Soft arm pouch pressures [psi]")
    pouch_sliders = [[None] * pouch_cmd.shape[1] for _ in range(pouch_cmd.shape[0])]

    def make_slider(label: str, command) -> tk.Scale:
        return tk.Scale(
            root, label=label, from_=MAX_PRESSURE_PSI, to=0.0, resolution=0.1,
            length=90, width=12, orient=tk.VERTICAL, command=command,
        )

    def set_segment(s: int, value: str) -> None:
        for slider in pouch_sliders[s]:
            slider.set(float(value))

    def set_pouch(s: int, k: int, value: str) -> None:
        pouch_cmd[s, k] = float(value)

    segment_sliders = []
    for s, label in enumerate(SEGMENT_LABELS):
        segment = make_slider(f"{label} all", lambda v, s=s: set_segment(s, v))
        segment.grid(row=0, column=s, padx=4, pady=(6, 10))
        segment_sliders.append(segment)
        # Rows follow the hanging arm: P1 (level0, at the mount) on top, P5 lowest.
        for k in range(pouch_cmd.shape[1]):
            pouch = make_slider(f"P{k + 1}", lambda v, s=s, k=k: set_pouch(s, k, v))
            pouch.grid(row=k + 1, column=s, padx=4)
            pouch_sliders[s][k] = pouch
    for segment in segment_sliders:
        segment.set(BASELINE_PSI)

    def reset() -> None:
        for s, segment in enumerate(segment_sliders):
            segment.set(BASELINE_PSI)
            set_segment(s, str(BASELINE_PSI))

    last_row = pouch_cmd.shape[1] + 1
    tk.Button(root, text=f"Reset all to {BASELINE_PSI:g} psi", command=reset).grid(
        row=last_row, column=0, columnspan=4, pady=6
    )
    status = tk.StringVar()
    tk.Label(root, textvariable=status, font=("TkFixedFont", 10), justify=tk.LEFT).grid(
        row=last_row + 1, column=0, columnspan=4, padx=6, pady=(0, 6), sticky="w"
    )
    return status


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dump-xml", metavar="PATH", help="write the generated MJCF and exit")
    parser.add_argument("--duration", type=float, default=None, help="close after N seconds")
    args = parser.parse_args()

    if args.dump_xml:
        xml = build_arm_xml(RobotCalibration.load().make_arm_config())
        with open(args.dump_xml, "w") as f:
            f.write(xml)
        print(f"Wrote {args.dump_xml}")
        return

    sim = make_sim(control_hz=CONTROL_HZ, seed=0)
    obs = sim.reset()
    home = obs["tip_pos"].copy()
    pouch_cmd = np.full((4, 5), BASELINE_PSI)

    root = tk.Tk()
    status = build_panel(root, pouch_cmd)
    viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
    start = time.monotonic()
    next_tick = start

    def tick() -> None:
        """Advance the plant in real time; Tk's event loop schedules this."""
        nonlocal obs, next_tick
        if not viewer.is_running() or (
            args.duration is not None and time.monotonic() - start >= args.duration
        ):
            root.quit()
            return
        # Catch up on any control steps that are due, then hand control back to Tk.
        while time.monotonic() >= next_tick:
            with viewer.lock():
                obs = sim.step(pouch_cmd.copy())
            next_tick += 1.0 / CONTROL_HZ
        viewer.sync()
        dx, dy, dz = (obs["tip_pos"] - home) * 1000
        status.set(
            f"t = {obs['time']:6.2f} s\n"
            f"tip Δ [mm]: x {dx:+6.1f}  y {dy:+6.1f}  z {dz:+6.1f}\n"
            f"measured psi: {np.array2string(obs['segment_pressures'], precision=2)}"
        )
        root.after(5, tick)

    root.protocol("WM_DELETE_WINDOW", root.quit)
    root.after(0, tick)
    root.mainloop()
    # close() only requests exit; wait for the viewer's render thread to finish
    # before interpreter shutdown terminates GLFW underneath it.
    viewer.close()
    for thread in threading.enumerate():
        if thread is not threading.main_thread():
            thread.join(timeout=5.0)
    root.destroy()
    sim.close()


if __name__ == "__main__":
    main()
