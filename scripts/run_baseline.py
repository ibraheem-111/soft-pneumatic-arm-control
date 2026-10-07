"""Reproduce the professor's PD baseline from the coursework notebook.

The controller class is executed straight from notebooks/soft_robotic_arm.ipynb
so this number always matches what the notebook defines. Uses the notebook's
benchmark settings: sim seed 2, trajectory seed 7, 100 Hz, 20 s.

Usage:
    uv run python scripts/run_baseline.py        # expected: 4.10 mm with soft-robotic-arm 0.4.2
"""

import json
from pathlib import Path

import numpy as np

from soft_robotic_arm import make_sim

NOTEBOOK = Path(__file__).resolve().parent.parent / "notebooks" / "soft_robotic_arm.ipynb"
CONTROL_HZ = 100.0
MAX_PRESSURE_PSI = 9.0
RUN_TIME_S = 20.0


def load_baseline_controller():
    cells = json.loads(NOTEBOOK.read_text())["cells"]
    source = next(
        "".join(c["source"]) for c in cells
        if c["cell_type"] == "code" and "class RandomTrajectoryPDController" in "".join(c["source"])
    )
    namespace = {"np": np, "MAX_PRESSURE_PSI": MAX_PRESSURE_PSI}
    exec(source, namespace)
    return namespace["RandomTrajectoryPDController"]


def main() -> None:
    controller = load_baseline_controller()()
    sim = make_sim(control_hz=CONTROL_HZ, seed=2)
    obs = sim.reset()
    references, tips = [], []
    for _ in range(int(RUN_TIME_S * CONTROL_HZ)):
        command = np.asarray(controller.compute(obs["time"], obs), dtype=float)
        if command.shape != (4, 5) or not np.all(np.isfinite(command)):
            raise ValueError("Controller must return a finite (4, 5) pouch-pressure matrix")
        obs = sim.step(command)
        references.append(controller.reference(obs["time"])[0])
        tips.append(obs["tip_pos"][:2].copy())
    errors = np.linalg.norm(np.asarray(references) - np.asarray(tips), axis=1)
    print(f"Planar tracking RMSE: {1000.0 * np.sqrt(np.mean(errors**2)):.2f} mm")


if __name__ == "__main__":
    main()
