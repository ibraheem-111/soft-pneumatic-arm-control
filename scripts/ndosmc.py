"""Qiao, Tao & Zhang (2024) SMC and NDOSMC in task space, plus a delay-aware variant.

Controllers for `make_calibrated_sim` (3 inputs, S2-S4), scored with the package's
own harness (`evaluate_controller`, `CircularTrackingTask`, seed 0). They use only
hardware signals (tip position/velocity, pressure sensors) and a model identified
from data by `scripts/task_space_model.py` (parameters in
`docs/figures/task_space_model.json`); nothing is read from the simulator's model.
The controllers are softarm.controllers.PaperSMC, PaperNDOSMC and DelayAwareNDOSMC on
softarm.models.TaskSpaceModel (M1 with the M2 parts); see softarm/controllers/smc.py.

  smc       paper eq. 5:  sigma = e, sign switching, eta = (dk|q| + dc|q'|)/c + eps
  ndosmc    paper eqs. 6-8, 12, 14: NDOB with gain l1 + 2 l2 |q|, sat boundary layer
  ndosmc-d  ndosmc on a Smith-predicted state with the identified actuator delay and
            lag (lag inverted), reference extrapolated by the delay

Usage:
    uv run python scripts/ndosmc.py                  # evaluate all controllers
    uv run python scripts/ndosmc.py --sweep          # small gain sweep, every run printed
"""

import argparse
from pathlib import Path

from soft_robotic_arm.evaluation import TrackingResult

from softarm.controllers import DelayAwareNDOSMC, PaperNDOSMC, PaperSMC
from softarm.evaluation import evaluate
from softarm.models import MODEL_JSON, TaskSpaceModel


def row(label: str, r: TrackingResult) -> str:
    return (f"{label:<44}{r.rmse_mm:>9.2f}{r.max_error_mm:>9.2f}"
            f"{r.phase_lag_s:>9.2f}{r.effort_psi:>9.2f}")


SWEEP_SMC = [dict(lam=lam, eps=eps) for lam in (1.0, 2.0, 4.0, 8.0) for eps in (0.1, 1.0)]
# l2 = l1/14 makes the nonlinear gain term equal l1 at |X| = 7 mm (the circle radius).
SWEEP_NDO = [dict(lam=lam, l1=l1, l2=l2f * l1, mu=mu, Phi=1.0, eps=1.0)
             for lam in (1.0, 2.0, 4.0) for l1 in (0.1, 0.2, 0.5, 1.0, 2.0, 5.0)
             for l2f in (0.0, 1.0 / 14.0) for mu in (0.1, 1.0)]
SWEEP_DELAY = SWEEP_NDO + [dict(lam=lam, l1=0.5, l2=0.0, mu=mu, Phi=1.0, eps=1.0)
                           for lam in (6.0, 8.0) for mu in (0.1, 1.0)]
DEFAULT_NDO = dict(lam=2.0, l1=1.0, l2=0.0, mu=1.0, Phi=1.0, eps=1.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--model", type=Path, default=MODEL_JSON)
    parser.add_argument("--sweep", action="store_true", help="run and print every gain setting")
    parser.add_argument("--lti", action="store_true", help="M1 with averaged (unscheduled) k, c")
    args = parser.parse_args()
    model = TaskSpaceModel.load(args.model, lpv=not args.lti)

    print(f"{'make_calibrated_sim seed 0, CircularTrackingTask':<44}{'RMSE':>9}{'max':>9}"
          f"{'lag s':>9}{'effort':>9}")
    smc_grid = SWEEP_SMC if args.sweep else [dict(lam=2.0, eps=0.1)]
    ndo_grid = SWEEP_NDO if args.sweep else [DEFAULT_NDO]
    for g in smc_grid:
        print(row(f"SMC eq5 {g}", evaluate(PaperSMC(model, **g))))
    for g in ndo_grid:
        print(row(f"NDOSMC {g}", evaluate(PaperNDOSMC(model, **g))))
    for g in (SWEEP_DELAY if args.sweep else ndo_grid):
        print(row(f"delay-aware {g}", evaluate(DelayAwareNDOSMC(model, **g))))
    # ablation: predictor + lag inverse with the observer switched off
    for lam in ((1.0, 2.0, 4.0, 8.0) if args.sweep else (4.0,)):
        for eps in (0.1, 1.0):
            g = dict(lam=lam, mu=1.0, eps=eps, observer=False)
            print(row(f"delay-aware, no observer {g}", evaluate(DelayAwareNDOSMC(model, **g))))
    print(row("paper SMC eq5, LTI model (lam 1, eps 0.1)",
              evaluate(PaperSMC(TaskSpaceModel.load(args.model, lpv=False), lam=1.0, eps=0.1))))


if __name__ == "__main__":
    main()
