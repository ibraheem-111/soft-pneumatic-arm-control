"""The comparison tests: six references, their sim seeds and when "after settling" starts.

    held-out circle    10 mm, 0.15 Hz, sim seed 5 (never used for tuning)
    scored circle      7 mm, 0.1 Hz, sim seed 0 (the package's CircularTrackingTask)
    triangle           vertices on a 7 mm circle (first at +x), 10 s per lap
    square             axis-aligned, vertices on a 7 mm circle (side 9.9 mm), 10 s per lap
    squiggle           smooth random: per axis 6 sines, 0.03-0.3 Hz, +-6 mm, path seed 101
    random waypoints   straight lines between random points (+-6 mm, 1-2.5 s), path seed 101

TUNE is the separate task controllers are tuned on (5 mm, 0.07 Hz circle, sim seed 5).
"""

from softarm.evaluation.references import circle, polygon, random_waypoints, smooth_random

TESTS = {  # name: (offset_fn(tau) -> m, sim seed, settle_s for "after settling", kind)
    "held-out circle": (circle(0.010, 0.15), 5, 1 / 0.15, "circle"),
    "scored circle": (circle(0.007, 0.10), 0, 10.0, "circle"),
    "triangle": (polygon(3, 0.007, 10.0), 0, 10.0, "polygon"),
    "square": (polygon(4, 0.007, 10.0, 45.0), 0, 10.0, "polygon"),
    "squiggle": (smooth_random(101), 0, 5.0, "open"),
    "random waypoints": (random_waypoints(101), 0, 5.0, "open"),
}

TUNE = dict(radius_m=0.005, frequency_hz=0.07, seed=5)
