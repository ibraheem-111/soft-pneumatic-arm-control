"""Reference motions on the package's CircularTrackingTask settings, as functions ref(home, t) -> m.

MOTIONS follow the RAS 556 project slides ("Simulation Results ... Motion"), at TASK's amplitude
and period (7 mm, 0.1 Hz):
  circle      the harness's own make_reference (a check of the loop)
  triangle    straight-line (Cartesian) interpolation through three vertices, V0 on +x, V1 upper
              left, V2 lower left (equilateral, circumradius 7 mm), one lap per 10 s at constant
              speed, starting at V0
  axial       the tip oscillates along one horizontal axis, 7 mm sin(2 pi 0.1 t), the other axis
              held at the settled position (along y and along x)
CIRCLE_TASKS: the scored circle and the held-out circle with their sim seeds (the tasks of
scripts/compare_controllers.py).
Used by scripts/motion_tests.py, scripts/compare_controllers.py and scripts/benchmark_unified.py.
"""

import numpy as np

from soft_robotic_arm.evaluation import CircularTrackingTask, make_reference

TASK = CircularTrackingTask()          # shared settings: rate, settle, duration, radius, frequency, baseline
CIRCLE_TASKS = {
    "scored (7 mm, 0.1 Hz, seed 0)": (CircularTrackingTask(), 0),
    "held-out (10 mm, 0.15 Hz, seed 5)": (CircularTrackingTask(radius_m=0.010, frequency_hz=0.15), 5),
}


def circle(home: np.ndarray, t: float) -> np.ndarray:
    return make_reference(home, t, TASK)


def triangle(home: np.ndarray, t: float) -> np.ndarray:
    angles = 2 * np.pi * np.arange(4) / 3
    verts = TASK.radius_m * np.column_stack([np.cos(angles), np.sin(angles)])   # V0, V1, V2, V0
    s = (TASK.frequency_hz * t % 1.0) * 3
    k = int(s)
    ref = home.copy()
    ref[:2] += verts[k] + (s - k) * (verts[k + 1] - verts[k])
    return ref


def axial(axis: int):
    def ref_fn(home: np.ndarray, t: float) -> np.ndarray:
        ref = home.copy()
        ref[axis] += TASK.radius_m * np.sin(2 * np.pi * TASK.frequency_hz * t)
        return ref
    return ref_fn


MOTIONS = {"circle (harness check)": circle, "triangle": triangle,
           "axial along y": axial(1), "axial along x": axial(0)}
