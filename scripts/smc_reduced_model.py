"""Reduced-model sliding-mode controller vs PD on the package's circular tracking task.

Port of Nikoletta's notebook `notebooks/SMC_Controller_withoutdisturbanceobserver.ipynb`
(sections 4-10). The controllers (softarm.controllers.PDController, SlidingModeController)
and the reduced-model derivation (softarm.models.reduced_model) follow the notebook; the only
additions are the `--gravity` option and the gain sweeps.

Plant and scoring are the package's own harness, not the `make_sim()` benchmark
in AGENTS.md: `soft_robotic_arm.evaluation.evaluate_controller` with the default
`CircularTrackingTask` (7 mm circle, 0.1 Hz, 5 s settle + 30 s tracked, 4.5 psi
baseline on S2-S4) on `make_calibrated_sim(control_hz=100, seed=0)` (3 inputs,
0.5 s transport delay, fitted regulator gain/bias, sealed S1 reservoir).

Reduced model (one bending mode per axis, pressure as a direct input):
    y'' + c y' + k y = b dp
The notebook builds k from the joint springs only. `--gravity` adds the
gravity stiffness of the hanging chain (finite difference of MuJoCo's
qfrc_bias), which the notebook leaves out. The notebook's control law uses the
absolute tip position in the k*y term; `--deviation` uses the deviation from the
tip position at the first call (the settled pose) instead. The notebook's
segment allocator uses each column's torque axis (-sin phi, cos phi) as if it
were the tip direction, which rotates x and y by 90 degrees (+1 psi of dp_x moves
the tip -2.07 mm in y); `--fix-allocator` uses the tip direction (cos phi, sin phi)
measured in the sim (S2 +y, S3 -x, S4 -y), and flips the sign of b, which the
by-hinge convention makes negative while +dp_x then moves the tip toward +x.

Usage:
    uv run python scripts/smc_reduced_model.py             # notebook model, final gains
    uv run python scripts/smc_reduced_model.py --sweep     # reproduce the notebook's PD and SMC sweeps
    uv run python scripts/smc_reduced_model.py --gravity   # same, with gravity in the reduced model
    uv run python scripts/smc_reduced_model.py --sweep --deviation [--gravity]
    uv run python scripts/smc_reduced_model.py --sweep --fix-allocator [--deviation] [--gravity]
"""

import argparse

import numpy as np

from soft_robotic_arm.evaluation import TrackingResult

from softarm.controllers import PDController, SlidingModeController
from softarm.evaluation import evaluate
from softarm.models import reduced_model


def row(label: str, r: TrackingResult) -> str:
    return (f"{label:<34}{r.rmse_mm:>10.2f}{r.max_error_mm:>10.2f}"
            f"{r.phase_lag_s:>10.2f}{r.effort_psi:>10.2f}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--gravity", action="store_true",
                        help="include gravity stiffness in the reduced model")
    parser.add_argument("--sweep", action="store_true",
                        help="run the notebook's PD and SMC gain sweeps")
    parser.add_argument("--deviation", action="store_true",
                        help="SMC uses tip deviation from the settled pose, not absolute position")
    parser.add_argument("--fix-allocator", action="store_true",
                        help="allocate along each segment's tip direction, not its torque axis")
    args = parser.parse_args()

    k, c, b = reduced_model(include_gravity=args.gravity)
    if args.fix_allocator:
        b = -b  # +dp along a tip direction moves the tip that way: b > 0
    omega0 = np.sqrt(k)
    zeta = c / (2 * omega0)
    print(f"Reduced model ({'springs + gravity' if args.gravity else 'springs only, as notebook'}):")
    print(f"  y'' + {c:.3f} y' + {k:.3f} y = {b:.4f} dp   "
          f"({omega0 / (2 * np.pi):.3f} Hz, zeta {zeta:.2f}, static gain {1000 * b / k:.2f} mm/psi)")
    print(f"  poles {np.roots([1.0, c, k]).round(3)}")
    home = evaluate(PDController(kp=0.0, kd=0.0, corrected_allocator=args.fix_allocator)).home
    print(f"  settled tip (circle centre) on make_calibrated_sim: "
          f"x {1000 * home[0]:.2f} mm, y {1000 * home[1]:.2f} mm, z {1000 * home[2]:.1f} mm")

    print(f"\n{'make_calibrated_sim, CircularTrackingTask':<34}{'RMSE mm':>10}{'max mm':>10}"
          f"{'lag s':>10}{'effort':>10}")
    pd_gains = [(250, 20), (100, 20), (500, 60), (500, 20)] if args.sweep else [(500, 20)]
    for kp, kd in pd_gains:
        print(row(f"PD kp={kp} kd={kd}", evaluate(PDController(
            kp=kp, kd=kd, corrected_allocator=args.fix_allocator))))
    smc_gains = ([(3.0, 1.0, 0.01), (3.0, 0.2, 0.02), (5.0, 0.2, 0.02), (8.0, 0.2, 0.02),
                  (5.0, 0.1, 0.02), (5.0, 0.35, 0.02)] if args.sweep else [(5.0, 0.2, 0.02)])
    for lam, K_switch, phi in smc_gains:
        smc = SlidingModeController(k, c, b, lam=lam, K_switch=K_switch, phi=phi,
                                    deviation=args.deviation, corrected_allocator=args.fix_allocator)
        print(row(f"SMC lam={lam} K={K_switch} phi={phi}", evaluate(smc)))


if __name__ == "__main__":
    main()
