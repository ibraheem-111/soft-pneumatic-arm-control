"""softarm: models, controllers and evaluation for the RAS556 soft pneumatic arm.

    softarm.constants     plant and harness constants, repository paths
    softarm.models        M1, M2, M3, F, OneMode (Model / LinearModel / FirstOrderModel)
    softarm.controllers   PD, SMC family, DelayMPC, KalmanDelayMPC, preview wrappers, presets
    softarm.evaluation    track(), references, the comparison TESTS, metrics
    softarm.plotting      shared figure style
    softarm.labels        series labels and the R_u grid shared by the comparison scripts
    softarm.reports       HTML helpers shared by the report builders

Any controller takes any model it can use, e.g.
    from softarm.models import M2, M3
    from softarm.controllers import DelayMPC, DelayAwareNDOSMC
    DelayMPC(M3.load(), ref_mm, horizon=100, r_u=1e-5)
    DelayAwareNDOSMC(M2.load(), lam=8, mu=1, eps=1, observer=False)
"""
