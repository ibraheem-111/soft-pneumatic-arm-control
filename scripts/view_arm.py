"""Interactive MuJoCo viewer for the coursework soft arm.

Runs the real ``make_sim()`` plant (pneumatic lag + pressure-to-joint forces)
inside MuJoCo's passive viewer, so you can watch the arm respond to pressures.

Keys (focus the viewer window):
    1 2 3 4   raise S1..S4 by 0.5 psi (all five pouches)
    Q W E R   lower S1..S4 by 0.5 psi
    0         reset every segment to the 2 psi baseline

Usage:
    uv run python scripts/view_arm.py
    uv run python scripts/view_arm.py --dump-xml soft_arm.xml   # write MJCF and exit
"""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from soft_robotic_arm import make_sim
from soft_robotic_arm.calibration import RobotCalibration
from soft_robotic_arm.model import build_arm_xml

CONTROL_HZ = 100.0
MAX_PRESSURE_PSI = 9.0
BASELINE_PSI = 2.0
STEP_PSI = 0.5

RAISE_KEYS = {ord("1"): 0, ord("2"): 1, ord("3"): 2, ord("4"): 3}
LOWER_KEYS = {ord("Q"): 0, ord("W"): 1, ord("E"): 2, ord("R"): 3}


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
    segment_cmd = np.full(4, BASELINE_PSI)

    def on_key(keycode: int) -> None:
        if keycode in RAISE_KEYS:
            segment_cmd[RAISE_KEYS[keycode]] += STEP_PSI
        elif keycode in LOWER_KEYS:
            segment_cmd[LOWER_KEYS[keycode]] -= STEP_PSI
        elif keycode == ord("0"):
            segment_cmd[:] = BASELINE_PSI
        else:
            return
        np.clip(segment_cmd, 0.0, MAX_PRESSURE_PSI, out=segment_cmd)
        print("command [S1 S2 S3 S4] psi:", segment_cmd)

    print(__doc__)
    dt = 1.0 / CONTROL_HZ
    start = time.monotonic()
    last_print = 0.0
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=on_key) as viewer:
        while viewer.is_running():
            tick = time.monotonic()
            with viewer.lock():
                obs = sim.step(segment_cmd.copy())
            viewer.sync()

            if obs["time"] - last_print >= 1.0:
                last_print = obs["time"]
                print(f"t={obs['time']:6.2f}s  tip [mm]={np.round(obs['tip_pos'] * 1000, 1)}  "
                      f"segment psi={np.round(obs['segment_pressures'], 2)}")
            if args.duration is not None and tick - start >= args.duration:
                break
            time.sleep(max(0.0, dt - (time.monotonic() - tick)))
    sim.close()


if __name__ == "__main__":
    main()
