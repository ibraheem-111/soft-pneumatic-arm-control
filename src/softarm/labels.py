"""Series labels and the R_u grid shared by the scripts that run, record and report the controller
comparisons. The labels key softarm.plotting.SERIES and the JSON outputs, so they must not drift."""

from softarm.constants import DT
from softarm.controllers.kalman_mpc import HORIZON as F_HORIZON

RU_GRID = (1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0)     # MPC weight on command changes, mm^2 / psi^2

SMC_KNOWN = "Delay-aware SMC (M2, known ref)"
NDOSMC_KNOWN = "Delay-aware NDOSMC (M2, known ref)"
SMC_L4 = "Delay-aware SMC (M2, known ref), λ = 4"       # NDOSMC's gain without the observer


def mpc_label(model: str, h: int, r_u: float | None = None) -> str:
    """e.g. "MPC (M2), horizon 100 steps = 1.00 s", with ", R_u = 10" for a fixed weight."""
    return f"MPC ({model}), horizon {h} steps = {h * DT:.2f} s" + ("" if r_u is None else f", R_u = {r_u:g}")


MPC_F = mpc_label("F", F_HORIZON)
