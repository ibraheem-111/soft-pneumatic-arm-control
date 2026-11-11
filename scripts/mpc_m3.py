"""Delay-compensated MPC on M3 (M2's actuator with second-order mechanics), reference known.

Runner: the experiment's code (runs, tuning, figures, outputs) is softarm.experiments.mpc_m3,
which notebooks/mpc_m3.ipynb also uses.

Usage:
    uv run python scripts/mpc_m3.py        # tune R_u, print it
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import matplotlib  # noqa: E402

matplotlib.use("Agg")

from softarm.experiments.mpc_m3 import main  # noqa: E402

if __name__ == "__main__":
    main()
