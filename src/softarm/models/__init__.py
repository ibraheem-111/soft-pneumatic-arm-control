"""Plant models of the soft arm. Every model loads its identified parameters itself:

    M1, TaskSpaceModel   rho-scheduled first-order mechanics on v = G u (+ M2 parts)   task_space.py
    M2                   actuator + first-order mechanics, state (x, p)               m2.py
    M3                   actuator + second-order mechanics, state (x, x', p)          m3.py
    F                    per-channel frequency-identified state space                 f.py
    OneMode              Nikoletta's one bending mode per plane axis                  one_mode.py
    physics              linearised MuJoCo plant (make_sim), ground truth for F      physics.py
"""

from softarm.models.base import FirstOrderModel, LinearModel, Model, lsim, plane_inputs
from softarm.models.f import F
from softarm.models.m2 import M2, load_m2_dict
from softarm.models.m3 import M3
from softarm.models.one_mode import OneMode, reduced_model
from softarm.models.task_space import M1, MODEL_JSON, ActuatorModel, AxisModel, TaskSpaceModel

__all__ = ["Model", "LinearModel", "FirstOrderModel", "lsim", "plane_inputs", "M1", "TaskSpaceModel",
           "AxisModel", "ActuatorModel", "MODEL_JSON", "M2", "load_m2_dict", "M3", "F", "OneMode",
           "reduced_model"]
