"""Evaluation: the tracking loop, references, the comparison tests and metrics.

    harness.py      track(), track_ref_mm(), evaluate(), settled_home(), oracle_for()
    references.py   circle, polygon, smooth_random, random_waypoints, ref_array
    tests.py        TESTS (the six comparison tests), TUNE
    metrics.py      score(), planar_metrics(), brief_score(), saturated_pct()
    brief_tasks.py  the five brief tasks (axial, open-loop circle, circles, triangle) and reach bounds
    motions.py      the motion tests (circle, triangle, axial) and the scored/held-out circle tasks
    recorded.py     the lab's recorded runs (upstream repository) replayed on the simulator
"""

from softarm.evaluation.harness import evaluate, oracle_for, settled_home, track, track_ref_mm
from softarm.evaluation.metrics import brief_score, planar_metrics, saturated_pct, score
from softarm.evaluation.references import circle, polygon, random_waypoints, ref_array, smooth_random
from softarm.evaluation.tests import TESTS, TUNE

__all__ = ["track", "track_ref_mm", "evaluate", "settled_home", "oracle_for", "score", "planar_metrics",
           "brief_score", "saturated_pct", "circle", "polygon", "smooth_random", "random_waypoints",
           "ref_array", "TESTS", "TUNE"]
