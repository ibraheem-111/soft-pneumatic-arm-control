"""Reference preview and measurement wrappers around a controller.

Ahead        passes the controller the true r(t + T) from a known reference array (T = its dead time)
Predicted    passes r(t + T) predicted from the reference's own history (RefPredictor)
RefPredictor predicts r(n+k) from the reference history: AR(2), detected period, or hold
NoisyTip     adds white noise to the tip the controller sees (scoring uses the true tip)
"""

from collections import deque

import numpy as np

from softarm.controllers.base import Controller


class RefPredictor:
    """Predicts r(n+k) from the reference's own history.

    mode "ar": AR(2) with intercept on samples `stride` apart over the last `window`
    samples (exact for a sinusoid plus offset), interpolated back to the control rate.
    It holds the last value until 5 coarse samples exist, or when the fitted AR poles
    lie outside the unit circle (a growing prediction).
    mode "periodic": as "ar" until the reference is seen to repeat, then
    r(n+k) = r(n+k-mP) with P the detected period (exact for any periodic reference,
    including corners an AR model cannot foresee). Every 0.5 s the period is searched
    over 1-15 s by comparing the last 2 s with the history P earlier; it is accepted
    when the mean squared mismatch is below 1e-6 of the signal's variance.
    mode "hold": r(n+k) = r(n), i.e. no preview."""

    def __init__(self, mode: str = "periodic", stride: int = 10, window: int = 400,
                 period_range: tuple[int, int] = (100, 1500), match: int = 200, every: int = 50):
        self.mode, self.stride, self.window = mode, stride, window
        self.period_range, self.match, self.every = period_range, match, every
        self.hist: deque = deque(maxlen=period_range[1] + match + 1)
        self.period: int | None = None

    def update(self, r: float) -> None:
        self.hist.append(r)
        if self.mode == "periodic" and len(self.hist) % self.every == 0:
            self._detect_period()

    def _detect_period(self) -> None:
        h = np.array(self.hist)
        lo, hi = self.period_range
        hi = min(hi, h.size - self.match)
        var = h.var()
        if hi < lo or var < 1e-18:
            return
        recent = h[-self.match:]
        cost = np.array([np.mean((recent - h[-self.match - P:-P]) ** 2) for P in range(lo, hi + 1)])
        best = int(np.argmin(cost))
        self.period = lo + best if cost[best] < 1e-6 * var else None

    def predict(self, start: int, length: int) -> np.ndarray:
        r_now = self.hist[-1]
        hold = np.full(length, r_now)
        if self.mode == "periodic" and self.period:
            h, P = np.array(self.hist), self.period
            k = np.arange(start, start + length)
            back = k - P * np.ceil(k / P).astype(int)          # index relative to now, always <= 0
            return h[h.size - 1 + back]
        recent = list(self.hist)[-(self.window + 1):]
        coarse = np.array(recent[::-1][::self.stride][::-1])
        if self.mode == "hold" or coarse.size < 5 or np.ptp(coarse) < 1e-9:
            return hold
        X = np.column_stack([np.ones(coarse.size - 2), coarse[1:-1], coarse[:-2]])
        c, a1, a2 = np.linalg.lstsq(X, coarse[2:], rcond=None)[0]
        if np.abs(np.roots([1.0, -a1, -a2])).max() > 1.0 + 1e-3:
            return hold                                       # growing prediction: do not trust it
        n_coarse = (start + length) // self.stride + 2
        z = [coarse[-2], coarse[-1]]
        for _ in range(n_coarse):
            z.append(c + a1 * z[-1] + a2 * z[-2])
        z = np.array(z[1:])                                   # z[0] = r(n), z[k] = r(n + k stride)
        return np.interp(np.arange(start, start + length), self.stride * np.arange(z.size), z)


class Ahead(Controller):
    """Passes the controller the reference N steps ahead (N = its dead-time steps, ctl.n_delay).

    ref_mm: the known reference, (n, 3) mm from the settled tip, sampled at the control rate."""

    def __init__(self, ctl, ref_mm: np.ndarray):
        self.c, self.d, self.k = ctl, ref_mm, 0

    def compute(self, t, obs, ref):
        k, n = self.k, self.c.n_delay
        self.k += 1
        return self.c.compute(t, obs, ref + (self.d[min(k + n, len(self.d) - 1)] - self.d[k]) / 1000.0)


class Predicted(Controller):
    """Passes the controller r(t + T) predicted from the reference history (no future knowledge).

    One RefPredictor (mode "periodic": AR(2) with intercept until the reference is seen to repeat,
    then the detected period replayed) per axis, on the reference in metres relative to the
    first reference's home (the settled tip). T = the controller's dead-time steps (n_delay)."""

    def __init__(self, ctl, mode: str = "periodic"):
        self.c = ctl
        self.pred = [RefPredictor(mode) for _ in range(3)]
        self.home: np.ndarray | None = None

    def compute(self, t, obs, ref):
        if self.home is None:
            self.home = np.asarray(obs["tip_pos"], float).copy()
        r = np.asarray(ref, float) - self.home
        ahead = np.empty(3)
        for j, p in enumerate(self.pred):
            p.update(r[j])
            ahead[j] = p.predict(self.c.n_delay, 1)[0]
        return self.c.compute(t, obs, self.home + ahead)


class NoisyTip(Controller):
    """Wraps a controller so that it sees tip position with added white noise (the
    simulator's tip sensor is noise-free; motion capture is not). Scoring still uses
    the true tip."""

    def __init__(self, inner, sd_m: float = 1e-4, seed: int = 0):
        self.inner, self.sd, self.rng = inner, sd_m, np.random.default_rng(seed)

    def compute(self, t: float, obs: dict, ref: np.ndarray) -> np.ndarray:
        noisy = dict(obs)
        noisy["tip_pos"] = np.asarray(obs["tip_pos"], dtype=float) + np.append(self.rng.normal(0, self.sd, 2), 0.0)
        noisy["tip_vel"] = np.asarray(obs["tip_vel"], dtype=float)
        return self.inner.compute(t, noisy, ref)
