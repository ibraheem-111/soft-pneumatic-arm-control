"""Reference trajectories as functions of tracking time: offset from the settled tip (m, (3,))."""

import numpy as np

from soft_robotic_arm.evaluation import CircularTrackingTask, make_reference

from softarm.constants import DT, HZ, PAD, TRACK_S


def circle(radius_m: float, freq: float):
    """The package's circular reference (starts on the -x side), centred on the settled tip."""
    task = CircularTrackingTask(radius_m=radius_m, frequency_hz=freq)
    return lambda tau: make_reference(np.zeros(3), tau, task)


def polygon(n: int, radius_m: float, lap_s: float, first_deg: float = 0.0):
    """Regular n-gon with vertices on radius_m, one lap per lap_s."""
    th = np.radians(first_deg) + 2 * np.pi * np.arange(n + 1) / n
    v = radius_m * np.column_stack([np.cos(th), np.sin(th)])

    def fn(tau):
        s = (tau / lap_s % 1.0) * n
        k = int(s)
        return np.append(v[k] + (s - k) * (v[k + 1] - v[k]), 0.0)
    return fn


def smooth_random(seed: int, span_mm: float = 6.0, f_lo: float = 0.03, f_hi: float = 0.3, n: int = 6):
    """Per axis a sum of n random sines (frequencies f_lo-f_hi Hz, random phases and weights),
    scaled to +-span_mm (same as notebooks/Soft_robotic_arm_SMC_M2.ipynb)."""
    g = np.random.default_rng(seed)
    f = g.uniform(f_lo, f_hi, (2, n)); ph = g.uniform(0, 2 * np.pi, (2, n)); a = g.uniform(0.3, 1.0, (2, n))
    raw = lambda t: (a * np.sin(2 * np.pi * f * np.asarray(t)[..., None, None] + ph)).sum(-1)  # noqa: E731
    tt = np.arange(0.0, TRACK_S, DT)
    scale = span_mm / 1000.0 / np.abs(raw(tt) - raw(0.0)).max(axis=0)
    return lambda tau: np.append((raw(tau) - raw(0.0)) * scale, 0.0)


def random_waypoints(seed: int, span_mm: float = 6.0, seg_lo: float = 1.0, seg_hi: float = 2.5):
    """Straight lines between random waypoints (+-span_mm), seg_lo-seg_hi s per segment."""
    g = np.random.default_rng(seed)
    times, pts = [0.0], [np.zeros(2)]
    while times[-1] < TRACK_S + PAD * DT + 1.0:
        times.append(times[-1] + g.uniform(seg_lo, seg_hi))
        pts.append(g.uniform(-span_mm, span_mm, 2) / 1000.0)
    pts = np.array(pts)
    return lambda tau: np.array([np.interp(tau, times, pts[:, 0]), np.interp(tau, times, pts[:, 1]), 0.0])


def ref_array(fn, track_s: float = TRACK_S, pad: int = PAD) -> np.ndarray:
    """Sample a reference function at the control rate, in mm, with `pad` extra samples for look-ahead."""
    n = int(round(track_s * HZ)) + pad
    return np.array([1000.0 * fn(k * DT) for k in range(n)])
