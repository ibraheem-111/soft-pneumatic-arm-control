"""Pressures, disturbance and error-space phase portraits for the two MPCs on the M2 comparison tests.

Runner: the experiment's code (runs, tuning, figures, outputs) is softarm.experiments.mpc_signals,
which notebooks/mpc_m3.ipynb also uses.

Usage:
    uv run python scripts/mpc_signals.py
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import matplotlib  # noqa: E402

matplotlib.use("Agg")

from softarm.experiments.mpc_signals import main  # noqa: E402

if __name__ == "__main__":
    main()
