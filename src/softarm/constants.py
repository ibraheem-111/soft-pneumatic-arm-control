"""Plant, harness and path constants shared by every model, controller and script.

The control rate, settle time, baseline and command limit are those of the package harness
(soft_robotic_arm.evaluation.CircularTrackingTask) on make_calibrated_sim().
"""

from pathlib import Path

HZ = 100.0                 # control rate (Hz)
DT = 1.0 / HZ              # control step (s)
BASE_PSI = 4.5             # settled command on S2-S4 (psi)
LIMIT_PSI = 9.0            # command limit used throughout the project (psi)
SETTLE_S = 5.0             # settle at BASE_PSI before tracking (s)
TRACK_S = 30.0             # tracked seconds per run
PAD = 400                  # reference samples kept beyond a run for look-ahead (4 s)

ROOT = Path(__file__).resolve().parents[2]          # repository root (src/softarm/constants.py)
FIG = ROOT / "docs" / "figures"
TASK_SPACE_JSON = FIG / "task_space_model.json"    # M1, M2, M3, actuator (scripts/task_space_model.py)
F_MODEL_JSON = ROOT / "models" / "sysid_calibrated_v2.json"   # F (scripts/sysid_frf.py)
