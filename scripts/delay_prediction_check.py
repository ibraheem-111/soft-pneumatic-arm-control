"""Is the delay prediction (Smith predictor) needed? Run each controller with and without it.

  NDOSMC (Smith)          scripts/ndosmc.py DelayAwareNDOSMC, known reference, Nikoletta's gains
  NDOSMC, no Smith        the same, with x_hat = x: the Smith correction x_m(t) - x_m(t-T) forced to 0
  MPC M2 (delay pred.)    compare_m2_controllers.M2MPC, horizon 100 steps = 1 s, R_u = 1e-5
  MPC M2, no delay pred.  the same, planning from the measured state as if no commands were in flight
                          (X(k+N) = X(k)); the reference window is still read N steps ahead
On the six tests of compare_m2_controllers.py (make_calibrated_sim, 100 Hz).

Usage:
    uv run python scripts/delay_prediction_check.py
"""

import multiprocessing as mp
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import benchmark_all_tasks as B  # noqa: E402
import benchmark_slide_tasks as BS  # noqa: E402
import compare_m2_controllers as C  # noqa: E402
import ndosmc as N  # noqa: E402


class NoSmith:
    """Ahead(DelayAwareNDOSMC) with the Smith correction removed, so the law sees the measured tip."""

    def __init__(self, ahead):
        self.a = ahead

    def compute(self, t, obs, ref):
        c = self.a.c
        c.x_hist[0] = c.x_model.copy()          # x_model(t) - x_model(t - T) = 0
        return self.a.compute(t, obs, ref)


class MPCNoPred(C.M2MPC):
    """M2MPC with X(k+N) replaced by the measured X(k)."""

    def compute(self, t, obs, ref):
        self.W = [np.zeros_like(w) for w in self.W]
        self.Apow = list(self.Apow)
        self.Apow[self.nd] = np.eye(6)
        return super().compute(t, obs, ref)


M = N.TaskSpaceModel.load()
G = dict(lam=4.0, l1=0.1, l2=0.0, mu=1.0, Phi=1.0, eps=1.0, exact_preview=True)
CTL = {
    "NDOSMC (Smith)": lambda d: BS.Ahead(N.DelayAwareNDOSMC(M, **G), d),
    "NDOSMC, no Smith": lambda d: NoSmith(BS.Ahead(N.DelayAwareNDOSMC(M, **G), d)),
    "MPC M2 (delay pred.)": lambda d: C.M2MPC(d, 100, 1e-5, M),
    "MPC M2, no delay pred.": lambda d: MPCNoPred(d, 100, 1e-5, M),
}


def run(job):
    test, name = job
    fn, seed, settle_s, kind = C.TESTS[test]
    d = C.ref_array(fn)
    res = B.track(CTL[name](d), lambda home, k: home + d[k] / 1000.0, int(round(C.TRACK_S * C.HZ)), seed=seed)
    s = C.score(1000 * (res["tips"] - res["home"]), 1000 * (res["refs"] - res["home"]), settle_s, res["cmds"], kind)
    return test, name, s["rmse_whole_mm"], s["rmse_settled_mm"]


def main() -> None:
    with mp.get_context("fork").Pool(12) as pool:
        out = pool.map(run, [(t, n) for t in C.TESTS for n in CTL])
    print("RMSE whole run / after settling (mm)")
    for t in C.TESTS:
        print(f"{t:<17}" + "  ".join(f"{n}: {w:.2f}/{s:.3f}" for tt, n, w, s in out if tt == t))


if __name__ == "__main__":
    main()
