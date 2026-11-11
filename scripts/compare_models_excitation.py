"""Run the model check (softarm.experiments.model_check) and write its figures and data.

Simulator and five models (One-mode, One-mode + gravity, M2, M3, F) get the same commands; see the
module docstring of softarm.experiments.model_check for the models, excitations and metrics.
Writes docs/figures/model_comparison.json (metrics, settings), model_comparison_data.npz (time
series), model_comparison_<excitation>[_<model>].png, model_comparison_rmse.png,
model_actuator_step.png and model_prediction_random_steps.png.

Usage:
    uv run python scripts/compare_models_excitation.py [--excitation random_steps chirp] [--seconds 60]
"""

import matplotlib

matplotlib.use("Agg")

from softarm.experiments.model_check import main  # noqa: E402

if __name__ == "__main__":
    main()
