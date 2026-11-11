"""Tracking metrics (planar x-y unless named otherwise, mm).

score()            the comparison tests' metrics (softarm.evaluation.tests): RMSE whole run and after
                   settling, shape error (RMS distance to the nearest point of the reference path,
                   densified 5x), max error, saturation, lag, and radius range (circles) or path size
                   (polygons, RMS distance from the centre)
planar_metrics()   planar and z RMSE, whole run and after `after_s`, saturation (recorded-run benchmark)
brief_score()      the brief tasks' metrics (scripts/benchmark_slide_tasks.py): adds max error, lag,
                   shape on the last lap's samples, size % by mean distance, radius range
These are three different metric sets, each reproducing the numbers already reported with it.
"""

import numpy as np

from soft_robotic_arm.evaluation import _phase_lag_seconds

from softarm.constants import HZ, LIMIT_PSI


def saturated_pct(cmds: np.ndarray) -> float:
    """% of command samples at 0 or the limit."""
    return float(100 * np.mean((cmds <= 0.0) | (cmds >= LIMIT_PSI)))


def score(tip: np.ndarray, ref: np.ndarray, settle_s: float, cmds: np.ndarray, kind: str) -> dict:
    """tip, ref: (n, 3) mm from home; kind: "circle", "polygon" or "open" (non-periodic)."""
    e = np.linalg.norm(tip[:, :2] - ref[:, :2], axis=1)
    k0 = int(round(settle_s * HZ))
    path = ref[k0:, :2] if kind == "open" else ref[-k0:, :2]      # periodic: one lap of the path
    dense = np.vstack([np.linspace(a, b, 5, endpoint=False) for a, b in zip(path[:-1], path[1:])])
    t = tip[k0:, :2]
    near = np.concatenate([np.sqrt(((t[i:i + 300, None] - dense[None]) ** 2).sum(-1)).min(1)
                           for i in range(0, len(t), 300)])
    out = dict(rmse_whole_mm=float(np.sqrt(np.mean(e ** 2))), rmse_settled_mm=float(np.sqrt(np.mean(e[k0:] ** 2))),
               shape_mm=float(np.sqrt(np.mean(near ** 2))), max_settled_mm=float(e[k0:].max()),
               saturated_pct=float(100 * np.mean((cmds <= 0.0) | (cmds >= 9.0))),
               lag_s=float(_phase_lag_seconds(ref[:, 1], tip[:, 1], HZ, max_lag_s=1.0)), settle_s=settle_s)
    if kind == "polygon":
        rms = lambda a: np.sqrt(np.mean(np.sum((a - a.mean(0)) ** 2, 1)))  # noqa: E731
        out.update(size_pct=float(100 * rms(t) / rms(ref[k0:, :2])))
    if kind == "circle":
        rad = np.linalg.norm(t - t.mean(0), axis=1)
        out.update(radius_min_mm=float(rad.min()), radius_max_mm=float(rad.max()),
                   radius_ref_mm=float(np.linalg.norm(ref[0, :2] - ref[:, :2].mean(0))))
    return out


def planar_metrics(err_mm: np.ndarray, cmds: np.ndarray | None = None, after_s: float = 5.0) -> dict:
    """err_mm (N, 3) = tip - reference. Planar and axial RMSE, whole run and after `after_s`."""
    late = np.arange(len(err_mm)) / HZ >= after_s
    planar = np.linalg.norm(err_mm[:, :2], axis=1)
    out = dict(planar_rmse_mm=float(np.sqrt(np.mean(planar**2))),
               planar_rmse_after_5s_mm=float(np.sqrt(np.mean(planar[late] ** 2))),
               z_rmse_mm=float(np.sqrt(np.mean(err_mm[:, 2] ** 2))),
               z_rmse_after_5s_mm=float(np.sqrt(np.mean(err_mm[late, 2] ** 2))))
    if cmds is not None:
        out["saturated_pct"] = float(100 * np.mean((cmds <= 0.0) | (cmds >= LIMIT_PSI)))
    return out


def brief_score(tip_mm: np.ndarray, ref_mm: np.ndarray, lap_s: float, cmds: np.ndarray | None = None) -> dict:
    """The brief tasks' metrics; lap_s: one lap (the "after" window starts there)."""
    err = tip_mm - ref_mm
    base = planar_metrics(err, cmds, after_s=lap_s)
    late = np.arange(len(err)) / HZ >= lap_s
    out = dict(planar_rmse_mm=base["planar_rmse_mm"], planar_rmse_after_lap_mm=base["planar_rmse_after_5s_mm"],
               z_rmse_mm=base["z_rmse_mm"], z_rmse_after_lap_mm=base["z_rmse_after_5s_mm"],
               max_planar_error_mm=float(np.linalg.norm(err[:, :2], axis=1).max()),
               max_planar_error_after_lap_mm=float(np.linalg.norm(err[late, :2], axis=1).max()),
               phase_lag_s=_phase_lag_seconds(ref_mm[:, 1], tip_mm[:, 1], HZ, max_lag_s=0.5 * lap_s))
    k0 = int(round(lap_s * HZ))
    path, tip, rf = ref_mm[-k0:, :2], tip_mm[k0:, :2], ref_mm[k0:, :2]  # last lap: open-loop refs start with a transient
    near = np.full(len(tip), np.inf)
    for i in range(0, len(tip), 500):  # chunked nearest-point search
        near[i:i + 500] = np.sqrt(((tip[i:i + 500, None] - path[None]) ** 2).sum(-1)).min(1)
    out["shape_rms_after_lap_mm"] = float(np.sqrt(np.mean(near ** 2)))
    out["size_pct_after_lap"] = float(100 * np.mean(np.linalg.norm(tip - tip.mean(0), axis=1))
                                      / np.mean(np.linalg.norm(rf - rf.mean(0), axis=1)))
    # size is an average: an oscillating path can average 100 %. The radius range shows that.
    rad = np.linalg.norm(tip - tip.mean(0), axis=1)
    out["radius_min_after_lap_mm"], out["radius_max_after_lap_mm"] = float(rad.min()), float(rad.max())
    if cmds is not None:
        out["saturated_pct"] = base["saturated_pct"]
    return out
