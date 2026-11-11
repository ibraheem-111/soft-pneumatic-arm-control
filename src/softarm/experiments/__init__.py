"""Experiments shared by a runner script (scripts/) and a notebook (notebooks/).

Each module holds one experiment's runs, tuning and figures; its script only calls main(), and
notebooks import the module directly (no sys.path):

    m2_comparison.py   controllers on M2 with the reference known: SMC, NDOSMC, MPC horizons
                       (scripts/compare_m2_controllers.py, notebooks/m2_controller_comparison.ipynb)
    mpc_signals.py     pressures, disturbance, offset and phase portraits of the MPCs
                       (scripts/mpc_signals.py, notebooks/mpc_m3.ipynb)
    mpc_m3.py          MPC on M3: R_u tuning and tracking figure (scripts/mpc_m3.py, notebooks/mpc_m3.ipynb)
    model_check.py     every model against the simulator under the same excitation
                       (scripts/compare_models_excitation.py, notebooks/model_check_excitation.ipynb)
"""
