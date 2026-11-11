"""Named controller configurations: the gains the project settled on, shared by the benchmarks.

baseline_set()      every controller built for the calibrated plant, at its chosen gains, with the
                    current reference only (scripts/compare_controllers.py and the benchmarks)
notebook_pd/smc()   the course notebook's PD and SMC, as written or with the corrected allocator
mpc_baselines()     PD and the best one-mode SMC with the corrected allocator (the F MPC's baselines)
"""

from softarm.controllers.pd import PDController
from softarm.controllers.smc import DelayAwareNDOSMC, PaperNDOSMC, PaperSMC, SlidingModeController
from softarm.models import M2, OneMode, TaskSpaceModel


def notebook_pd(corrected: bool, **kw) -> PDController:
    return PDController(corrected_allocator=corrected, **kw)


def notebook_smc(corrected: bool, **kw) -> SlidingModeController:
    """The notebook SMC on the one-mode model (springs only); b flipped with the corrected allocator."""
    return SlidingModeController.from_model(OneMode(include_gravity=False), corrected_allocator=corrected, **kw)


def baseline_set() -> list:
    """(label, family, make()) for every controller of scripts/compare_controllers.py."""
    m1, m2 = TaskSpaceModel.load(), M2.load()
    return [
        ("PD, notebook allocator", "notebook", lambda: notebook_pd(False, kp=500.0, kd=20.0)),
        ("PD, corrected allocator", "notebook", lambda: notebook_pd(True, kp=500.0, kd=20.0)),
        ("SMC (notebook model), as written", "notebook",
         lambda: notebook_smc(False, lam=5.0, K_switch=0.2, phi=0.02)),
        ("SMC (notebook model), corrected allocator + deviation, λ=8", "notebook",
         lambda: notebook_smc(True, lam=8.0, K_switch=0.2, phi=0.02, deviation=True)),
        ("Paper SMC, eq. 5 (M1)", "paper", lambda: PaperSMC(m1, lam=1.0, eps=0.1)),
        ("Paper NDOSMC, eq. 12 (M1)", "paper",
         lambda: PaperNDOSMC(m1, lam=1.0, l1=0.5, l2=0.0, mu=0.1, Phi=1.0, eps=1.0)),
        ("Delay-aware SMC, no observer (M2)", "delay",
         lambda: DelayAwareNDOSMC(m2, lam=8.0, mu=1.0, eps=1.0, observer=False)),
        ("Delay-aware NDOSMC (M2)", "delay",
         lambda: DelayAwareNDOSMC(m2, lam=4.0, l1=0.1, l2=0.0, mu=1.0, Phi=1.0, eps=1.0)),
    ]


def mpc_baselines() -> dict:
    """PD and the best SMC from the one-mode notebook model, corrected allocator."""
    return {
        "PD kp=500 kd=20": lambda: notebook_pd(True, kp=500, kd=20),
        "SMC lam=5 K=0.35 phi=0.02, deviation": lambda: notebook_smc(
            True, lam=5.0, K_switch=0.35, phi=0.02, deviation=True),
    }
