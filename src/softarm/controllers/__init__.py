"""Controllers, all with the harness interface compute(t, obs, ref) -> (3,) S2-S4 psi.

Which models each controller accepts (checked at construction):

    PDController                     none (task-space PD)                        pd.py
    SlidingModeController            OneMode            (Nikoletta's SMC)        smc.py
    PaperSMC, PaperNDOSMC            M1 (TaskSpaceModel)                         smc.py
    DelayAwareNDOSMC                 any FirstOrderModel: M2, TaskSpaceModel     smc.py
    DelayMPC                         any LinearModel with a measured state and
                                     segment inputs: M2, M3                       mpc.py
    KalmanDelayMPC                   F                                           kalman_mpc.py

Wrappers (preview.py): Ahead (known r(t+T)), Predicted (r(t+T) from history), NoisyTip.
Named configurations at the project's chosen gains: presets.py.
"""

from softarm.controllers.base import Controller
from softarm.controllers.kalman_mpc import KalmanDelayMPC, known_reference
from softarm.controllers.mpc import DelayMPC
from softarm.controllers.observers import NDOB
from softarm.controllers.pd import PDController, segment_allocator
from softarm.controllers.preview import Ahead, NoisyTip, Predicted, RefPredictor
from softarm.controllers.smc import DelayAwareNDOSMC, NotebookSMC, PaperNDOSMC, PaperSMC, SlidingModeController

__all__ = ["Controller", "PDController", "segment_allocator", "SlidingModeController", "NotebookSMC",
           "PaperSMC", "PaperNDOSMC", "DelayAwareNDOSMC", "NDOB", "DelayMPC", "KalmanDelayMPC",
           "known_reference", "Ahead", "Predicted", "NoisyTip", "RefPredictor"]
