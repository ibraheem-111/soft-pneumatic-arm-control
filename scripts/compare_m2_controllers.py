"""Compare controllers built on the identified M2 model, with the reference known in advance.

Runner: the experiment's code (runs, tuning, figures, outputs) is softarm.experiments.m2_comparison,
which notebooks/m2_controller_comparison.ipynb also uses.

Usage:
    uv run python scripts/compare_m2_controllers.py            # tune R_u, run, write (about 30 s)
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import matplotlib  # noqa: E402

matplotlib.use("Agg")

from softarm.experiments.m2_comparison import main  # noqa: E402

if __name__ == "__main__":
    main()
