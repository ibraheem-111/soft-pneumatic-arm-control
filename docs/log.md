# Project log

Newest first. Record decisions, findings and open questions here so the next
person (or agent) can pick up without asking. Keep entries short and dated.

## 2026-11-11

- Second submission notebook `notebooks/Soft_robotic_arm_MPC_M2sim.ipynb`, identical to the M3sim
  one except the model: M2's first-order mechanics identified on `make_sim` (a 4.66/4.69 1/s, H ≈
  1.84 mm/psi), R_u = 0.1 tuned on the same three non-test paths (mean 0.464 mm vs 0.404 for M3sim).
  Instructor's example 0.121 mm (M3sim 0.090, template PID 1.259); beats the PID on all 12
  trajectories; waypoints 0.19-0.30 mm vs 0.03-0.05 for M3sim. Both notebooks come from
  `scripts/build_submission.py` (M2sim by exact text replacements of the M3sim blocks).
- M2 controller comparison, MPC signals and the MPC on M3 moved into `softarm.experiments`
  (`m2_comparison`, `mpc_signals`, `mpc_m3`); `scripts/compare_m2_controllers.py`, `mpc_signals.py`
  and `mpc_m3.py` are thin runners, and `notebooks/m2_controller_comparison.ipynb` and
  `mpc_m3.ipynb` import the modules without sys.path. Rule (AGENTS.md): notebooks never add
  `scripts/` to sys.path. Outputs of the three scripts and both reports identical to before; both
  notebooks execute.
- Model check moved into `softarm.experiments.model_check` (excitations, simulator run, metrics,
  0.5 s-ahead prediction, actuator step fit, static gains, figures, runner);
  `scripts/compare_models_excitation.py` only calls its `main()`, and
  `notebooks/model_check_excitation.ipynb` imports it without sys.path. Outputs byte-identical
  (json, npz, every figure); the notebook executes without errors.
- Repository restructured into the installable package `src/softarm/` (Ibraheem's request:
  models and controllers each in one place, interchangeable through base classes). Models
  (`softarm.models`: `Model`, `LinearModel`, `FirstOrderModel`; M1, M2, M3, F, OneMode,
  physics), controllers (`softarm.controllers`: `Controller`; PD, SMC family, `DelayMPC`,
  `KalmanDelayMPC`, preview wrappers, presets), evaluation (`softarm.evaluation`: `track`,
  references, `TESTS`, metrics, brief tasks, motions, recorded runs), `constants`,
  `plotting`, `labels`, `reports`. Every script now imports `softarm` and no script imports
  another. `DelayMPC` takes any `LinearModel` with a measured state (M2, M3);
  `DelayAwareNDOSMC` any `FirstOrderModel` (M2, M1's container). Regression: every script
  rerun before and after; outputs compared leaf by leaf (see the restructure commit).
  Old → new names:
  - `ndosmc.TaskSpaceModel/AxisModel/ActuatorModel` → `softarm.models` (also `M1`); M2 parts → `softarm.models.M2`
  - `ndosmc.PaperSMC/PaperNDOSMC/DelayAwareNDOSMC/NDOB` → `softarm.controllers`
  - `smc_reduced_model.reduced_model` → `softarm.models.reduced_model` / `OneMode`;
    `PDController`, `SlidingModeController` → `softarm.controllers` (the `FIX_ALLOCATOR` global
    is now the `corrected_allocator` argument)
  - `compare_m2_controllers.M2MPC(ref_mm, h, r_u)` and `mpc_m3.DelayMPC/make_m3_mpc` →
    `softarm.controllers.DelayMPC(model, ref_mm, h, r_u)`; `mpc_m3.m3_matrices` → `M3.load().continuous()`
  - `mpc_controller.DelayMPCController/AxisController/RefPredictor/NoisyTip/baselines` →
    `softarm.controllers.KalmanDelayMPC`, `.kalman_mpc.AxisController`, `.RefPredictor`, `.NoisyTip`,
    `.presets.mpc_baselines`; `compare_m2_controllers.mpc_f` / `benchmark_unified.mpc_oracle` →
    `softarm.controllers.known_reference`
  - `benchmark_slide_tasks.Ahead` / `benchmark_unified.Predicted` → `softarm.controllers`
  - `compare_controllers.controllers()` → `softarm.controllers.presets.baseline_set()`
  - `benchmark_all_tasks.track/metrics` → `softarm.evaluation.track/planar_metrics`;
    `benchmark_slide_tasks.score/load_m2` → `softarm.evaluation.brief_score` / `softarm.models.load_m2_dict`
  - `compare_m2_controllers.TESTS/score/ref_array/circle/polygon/smooth_random/random_waypoints` →
    `softarm.evaluation`; `compare_models_excitation.simulate_*` → each model's `.simulate()`
  - `plant_model.equilibrium/linearize` → `softarm.models.physics`; `scripts/plot_style.py` →
    `softarm.plotting`
  The scripts keep their names, CLIs and outputs. Log entries before this date cite the old names.
- Professor's new template (`notebooks/soft_robotic_arm_v2.ipynb`, copied unchanged): the assessed
  plant is `make_sim` (seed 2, 20 s from reset, no settle), all four segments active, interface
  `student_controller(reference, obs)` -> finite (4, 5) pouch psi in [0, 9] plus
  `reset_controller()`; `reference(t)` may be queried ahead. Measured on `make_sim`: no dead time, τ
  = 0.60 s, gain 1.00, +1 psi on any segment moves the tip 1.81 mm toward it at every bias 1-8 psi,
  reach ±16 mm per axis, tip at the origin at reset with pouches at 0 psi. The professor allowed
  either plant.
- Submission notebook `notebooks/Soft_robotic_arm_MPC_M3sim.ipynb` (built by
  `scripts/build_submission.py`, standalone): model re-identified on `make_sim` (second order per
  axis: ω 7.08/7.10 rad/s, ζ 0.70, H ≈ 1.79 mm/psi; free-run 0.07-0.14 mm vs 0.17-0.42 for first
  order), MPC reading `reference(t)` 1 s ahead, R_u = 0.01 tuned on three paths not used for
  testing. Instructor's example: 0.090 mm RMSE vs 1.259 mm for the template PID; beats the PID on
  all 12 test trajectories. A `make_calibrated_sim` section (adapted evaluator: rows S2-S4 averaged)
  runs our M2 MPC: 1.8-2.9 mm vs PID 8.6-9.2 mm; that plant reaches only +4.2 mm in x (S1 sealed),
  so the 10 mm references are partly unreachable.

## 2026-11-10

- Colab submission notebooks `notebooks/Soft_robotic_arm_MPC_M2.ipynb` and
  `Soft_robotic_arm_MPC_M3.ipynb`, generated by `scripts/build_mpc_colab.py`: standalone (install
  soft-robotic-arm 0.4.2, parameters inlined), MPC with the known trajectory, R_u tuning, official
  `evaluate_controller` on the scored and held-out circles, six tests, three random seeds, plots.
  Executed outside the repo: no errors. Official scored-task RMSE: M2 MPC 1.169 mm, M3 MPC 1.168 mm.
- M3 MPC is not reliably better on random paths (correction to the 2026-11-09 entry, which used path
  seed 101 only): squiggle seed 103 0.302 mm with 66 % of commands saturated (M2 MPC 0.029 mm),
  waypoints seed 102 0.281 mm, 51 % saturated (M2 0.231 mm). Its unconstrained optimum asks for more
  than the 0–9 psi valves allow.
- The delay prediction is necessary (`scripts/delay_prediction_check.py`). NDOSMC without the Smith
  correction (x̂ = x): RMSE after settling 1.87–2.29 mm on every test, against 0.05–0.90 with it. M2
  MPC without the delay prediction (X(k+N) = X(k)): unstable, 9.4–11.3 mm, against 0.03–0.91.

## 2026-11-09

- MPC on M3 (`scripts/mpc_m3.py`, notebook `notebooks/mpc_m3.ipynb`): M2MPC's law on M3, state (y,
  ẏ, p) with ẏ from the differenced tip, horizon 100 steps = 1 s, R_u tuned on the tuning circle to
  1e-5 (0.037 mm there vs 0.015 for M2). RMSE after settling, M2 / M3 / F MPC (mm): held-out 0.905 /
  0.906 / 0.916, scored 0.029 / 0.074 / 0.057, triangle 0.129 / 0.069 / 0.066, square 0.124 / 0.069
  / 0.068, squiggle 0.071 / 0.041 / 0.063, waypoints 0.167 / 0.105 / 0.086. M3 saturates far more
  (6–46 % of commands vs 1–23 % for M2, 0–19 % for F). The velocity estimate is not the limit: the
  simulator's tip_vel gives the same tuning score; ẏ = 0 scores 0.030. One rerun moved M3's square
  by 0.001 mm (0.070 to 0.069); cause not checked.
- MPC counterpart of the SMC page: `docs/reports/mpc-on-m2-and-f.html`
  (`scripts/report_mpc_explained.py`), with pressures, actual disturbance, each MPC's offset
  estimate and error-space phase portraits per test (`scripts/mpc_signals.py`,
  `docs/figures/mpc_signals_*.png`, `mpc_phase_*.png`). Logged runs reproduce the comparison RMSEs
  exactly. Observed: the M2 MPC's commands jitter (R_u = 1e-5, near model inverse) where F's are
  smooth; on the triangle the M2 MPC's own predicted error stays within ±0.07 mm while its real
  error is 0.129 mm (model error its offset w does not catch), against ±0.25 mm predicted and 0.066
  mm real for F.
- SMC on M2 explained (`docs/reports/smc-on-m2.html`, built by `scripts/report_smc_explained.py`)
  with pressures, disturbance and error-space phase portraits per test (`scripts/smc_signals.py`,
  `docs/figures/smc_signals_*.png`, `smc_phase_*.png`).
- Correction to the 2026-11-08 observer finding: the NDOSMC's switching gain is η = Φe^(−l₁t) + ε/l₁
  → 10, not 1. The SMC at λ = 4 with η = 10 and no observer matches the NDOSMC on all six tests
  (e.g. waypoints 0.273 vs 0.274 mm, square 0.210 vs 0.213 mm after settling), so the gain over the
  λ = 4, η = 1 SMC comes from η, not from d̂. With l₁ = 0.1 the observer is a 10 s filter (d̂˙ =
  l₁(d − d̂)); its d̂ is about a fifth of the actual disturbance (RMS ≈ 0.1 vs 0.6 mm/s). Measured
  with `scripts/smc_signals.py`.
- Confirmed the correction above independently (`DelayAwareNDOSMC`, exact preview, all six M2
  comparison tests): SMC at λ = 4 with η = 10, no observer, whole-run RMSE 1.211 / 1.956 / 1.014 /
  1.051 / 0.311 / 0.273 mm vs NDOSMC 1.206 / 1.948 / 1.012 / 1.049 / 0.308 / 0.274 (scored, held-out,
  triangle, square, squiggle, waypoints). The 2026-11-08 model-check entry's "in closed loop the
  observer does help" is wrong for the same reason; the model-check notebooks and
  `docs/reports/model-comparison.html` now say so.

## 2026-11-08

- Dropped the Taylor-preview SMC from the M2 controller comparison (figures, tables, notebook,
  report) and from the controller diagrams: we no longer use it; the reference is known in advance.
  Its result stays recorded above (triangle 1.456 mm, 119 % size, vs 0.193 mm with the known
  reference).
- Controller block diagrams: `docs/diagrams/controllers.drawio` (draw.io, one page per controller in
  the M2 comparison: SMC known ref, NDOSMC, SMC Taylor preview, MPC on M2, MPC on F), generated from
  `scripts/draw_controller_diagrams.py` and drawn from the code, not from the paper.
- `notebooks/model_comparison_colab.ipynb`: standalone model comparison for Google Colab (installs
  soft-robotic-arm 0.4.2 and scipy if missing; all model parameters inlined, no repo imports). Checked
  by running it in a fresh venv outside the repo: same numbers as `compare_models_excitation.py`.

- Model comparison report: `scripts/report_model_comparison.py` -> `docs/reports/model-comparison.html`
  (artifact https://claude.ai/artifact/MrteR559Xf4EBi69sWjqNs). Figures reworked for red-green colour
  blindness: no green, plain model names on plots, one overview with all models plus one figure per
  model against the simulator (`model_comparison_<excitation>_<model>.png`), RMSE bars, actuator step
  and prediction figures.

- M2 controller comparison: added the delay-aware SMC at λ = 4 without the observer (suggested by
  the models session), so the NDOSMC row differs from it only by the observer. After settling, λ = 4
  without → with observer (mm): held-out 0.938 → 0.904, scored 0.046 → 0.050, triangle 0.225 →
  0.196, square 0.284 → 0.213, squiggle 0.086 → 0.095, waypoints 0.374 → 0.274; whole-run RMSE
  improves on all six. Report now built by `scripts/report_m2_controllers.py`.
- Actuator identified blind from a +1 psi step on S3 (measured pouch pressure,
  `compare_models_excitation.identify_actuator`): first rise 0.50 s, least-squares T = 0.490 s,
  tau = 0.599 s, G = 1.098 psi/psi, matching the assumed 0.5 s / 0.6 s. M2 + NDO scored on predicting
  the tip one dead time (0.5 s) ahead from the measured tip (a free run gives an observer nothing to
  observe): RMSE x / y / z mm, steps: hold 1.36 / 1.98 / 0.60, M2 0.180 / 0.218 / 0.038, M2 + NDO
  (l1 = 0.1) 0.187 / 0.222 / 0.069; chirp: M2 0.238 / 0.417 / 0.029, + NDO 0.244 / 0.423 / 0.099.
  d_hat is a slow drift (level mismatch of M2's absolute equations), which the Smith predictor already
  has from the measurement. In closed loop the observer does help: delay-aware NDOSMC at lam = 4,
  observer on vs off, whole-run RMSE mm: scored circle 1.286 -> 1.206, held-out 2.047 -> 1.948,
  triangle 1.069 -> 1.012, square 1.125 -> 1.049, squiggle 0.361 -> 0.308, waypoints 0.363 -> 0.274
  (the compare_m2_controllers.py SMC row uses lam = 8, so its gap to the NDOSMC row mixes gain and
  observer). Notebook gained sections on S1-S4 and excitations, the actuator model, and the NDO.

- M2 controller comparison moved into `notebooks/m2_controller_comparison.ipynb` (code stays in
  `scripts/compare_m2_controllers.py`). Added a triangle and a square (vertices on 7 mm, 10 s/lap,
  seed 0), the MPC on F (`mpc_controller.DelayMPCController`, ρ = 1e-5, 3 s horizon) given the true
  future reference, and the same delay-aware SMC with the Taylor preview. RMSE after the first lap,
  triangle / square (mm): SMC known ref 0.193 / 0.225, MPC on M2 0.129 / 0.124, MPC on F 0.066 /
  0.068, SMC Taylor preview 1.456 / 1.555 (path size 119 % / 118 %). The SMC's earlier overshoot
  came from the Taylor preview, not the gain: same λ = 8 with the known reference gives 102 %. MPC
  on F is also best on the random waypoints (0.086 vs 0.167 mm on M2).
- Model check extended to five models. Added one-mode + gravity (Nikoletta's derivation with the
  chain's gravity stiffness, `reduced_model(True)`, k = 51.45; our review, not in her notebooks) and
  M3 (M2's actuator + second-order mechanics). Checked that the one-mode model used is her latest
  (Copy_of_soft_robotic_arm_SMC_Smith-2.ipynb cell 16: k = 10.6024, c = 9.2309, b = -0.08204, same
  as `reduced_model(False)`). Free-run RMSE x / y, mm, steps | chirp: one-mode + gravity 2.64 / 3.89
  | 2.86 / 5.03; M3 0.08 / 0.06 | 0.07 / 0.08 (z 0.09 | 0.11, same as M2). Static gain x per psi
  on S3: simulator 2.01, one-mode 7.74, + gravity 1.54, M2 1.99, M3 1.95, F 1.96 mm/psi. Gravity
  fixes most of the gain error but the one-mode models still have no delay or lag. Figures split
  per axis into derived vs identified panels; the notebook ends with a full error table (RMSE,
  max, bias, fit, static gains).

- Model check (`scripts/compare_models_excitation.py`, notebook `notebooks/model_check_excitation.ipynb`):
  same commands into `make_calibrated_sim` (seed 0) and three models run open loop. Free-run RMSE
  x / y / z, mm: random steps (seed 31, 60 s) one-mode 7.33 / 11.11 / -, M2 0.12 / 0.14 / 0.08,
  F 0.08 / 0.09 / -; chirp 0.02-1 Hz one-mode 7.86 / 14.47 / -, M2 0.15 / 0.25 / 0.11, F 0.08 /
  0.08 / -. The one-mode model is about 4x too large (static gain 7.7 vs about 2.0 mm/psi: no
  gravity stiffness) and has no delay or lag. Data: docs/figures/model_comparison.json, .npz.
  JupyterLab + ipykernel added as dev dependencies (`uv run jupyter lab`).

## 2026-11-08

- `scripts/compare_m2_controllers.py`: controllers on M2 with the reference known in advance:
  delay-aware SMC, delay-aware NDOSMC (Nikoletta's gains) and a new MPC on M2 (`M2MPC`: delay
  predictor, model-error correction, closed form, horizons 25/100/300 steps = 0.25/1/3 s).
  RMSE after settling: MPC 0.029 (scored), 0.071 (squiggle), 0.167 (waypoints), 0.905 mm
  (held-out); SMC 0.034 / 0.091 / 0.295 / 0.924. With R_u tuned on a separate circle
  (1e-5, edge of the grid; 1e-5..1e-1 within 0.002 mm) the horizon has no effect; at
  R_u = 10 the 0.25 s horizon is worse and 1 s = 3 s (waypoints 0.137 mm, better than tuned).
  Held-out circle is reach-limited in x (8.2 of 10 mm for every controller).
  Page: docs/reports/m2-controller-comparison.html.

## 2026-11-06

- Professor: final test uses irregular random trajectories, known in advance. Added §6 to
  `notebooks/Soft_robotic_arm_SMC_M2.ipynb`: smooth random (6 sines, 0.03-0.3 Hz, ±6 mm) and
  random waypoints (straight lines, 1-2.5 s segments, ±6 mm), seeds 101-103, sim seed 0, 30 s.
  Delay-aware NDOSMC (same gains), RMSE after 5 s: Taylor estimate 0.22-0.41 (smooth) /
  2.05-2.87 mm (waypoints); known trajectory (exact_preview + KnownTrajectory wrapper)
  0.06-0.10 / 0.27-0.36 mm. The notebook's track() reproduces evaluate_controller (1.21 mm).
- `notebooks/Soft_robotic_arm_SMC_M2.ipynb` (Nikoletta): self-contained submission notebook for
  the delay-aware NDOSMC on M2 (model values embedded). Rerun here: 1.21 mm whole run, 0.102 mm
  after 5 s, held-out 1.97 mm, identical to `compare_controllers.py`. Its "MPC 0.92 mm did not
  reproduce" note compares two different numbers: 0.92 mm is the MPC's held-out-circle RMSE
  after the first lap (results slide), 1.22 mm its scored-circle whole-run RMSE; both reproduce
  (`benchmark_unified.json`).

## 2026-10-28

- Professor (in class): the robot accepts up to 15 psi; we should probably limit commands to
  10 psi. Noted only; all code and results still use the 0-9 psi clip.
- Study guide `docs/reports/controller-study-guide.html`: every model (one-mode, M1, M2, M3, F)
  and controller (PD, notebook SMC, paper SMC/NDOSMC, delay-aware SMC with Taylor / exact /
  RefPredictor preview, MPC) derived step by step with file:line references, and the full
  `benchmark_unified.json` tables (generated from the JSON). Deck slide 6 now shows six
  controllers on five tests, RMSE after the first lap, from the same JSON.
- Unified benchmark (`uv run python scripts/benchmark_unified.py`, 156 runs, 2 min 45 s on 10
  workers; every run's gains, seed, test parameters, metrics and run time in
  `docs/figures/benchmark_unified.json`, heat maps in `benchmark_unified.png`): 12 controllers
  (compare_controllers' 7, mpc_controller's SMC baseline, MPC with predicted and with true future
  reference, delay-aware SMC with exact r(t+T), and new: delay-aware SMC fed r(t+T) by
  mpc_controller's `RefPredictor`) × 13 tests (scored and held-out circles, motion_tests'
  triangle / axial y / axial x, the 5 brief-scaled tasks, the 3 recorded robot runs), one harness
  loop (`benchmark_all_tasks.track`), `benchmark_slide_tasks.score()` for all. Reproduces every
  published number: compare_controllers, benchmark_slide_tasks and benchmark_all_tasks JSONs to
  1e-15 mm; mpc_controller 1.2203 (printed 1.220); motion_tests MPC triangle / axial y / axial x
  1.5098 / 0.4796 / 0.5114. Only difference: the exact-preview SMC's saturation % on the brief
  tasks (here the reference runs on past the end, which changes only the last 0.5 s of commands;
  tracking metrics identical). Planar RMSE after the first lap (mm), scored circle / held-out /
  MT triangle / brief circle 4.3 mm / brief triangle / recorded circular: MPC (predicted ref)
  0.057 / 0.92 / 0.62 / 0.15 / 0.71 / 3.57; delay-aware SMC (Taylor) 0.090 / 0.97 / 1.46 / 2.35 /
  1.99 / 1.56; delay-aware SMC + RefPredictor 0.034 / 0.92 / 1.78 / 0.55 / 2.35 / 2.07; paper SMC
  0.87 / 3.03 / 1.47 / 8.18 / 1.97 / 2.62; with true future reference (diagnostics): MPC 0.057 /
  0.92 / 0.066 / 0.15 / 0.084 / 0.093, delay-aware SMC 0.034 / 0.92 / 0.19 / 0.54 / 0.30 / 0.72.
  Findings: the MPC is best on synthetic periodic tasks but behind the paper and delay-aware SMCs
  on the recorded runs (3.1-4.5 mm vs 1.1-2.7): the recorded mocap path never repeats exactly, so
  RefPredictor never detects a period there (checked: 0 of 18000 steps on x and y, all three runs)
  and stays on AR(2); the Taylor-preview SMCs win there (1.1-1.6 mm). The RefPredictor SMC equals the
  exact-preview SMC once the period is detected (prediction error 6e-15 mm after 12 s on the MT
  triangle, commands identical to 1e-9 psi after 25 s), but on the triangle AR(2) before
  detection drives it into 40-54 % saturation and it needs ~3 s to recover, which the
  after-first-lap RMSE includes (MT triangle 4.05 mm RMS 10-12.5 s, 0.14 mm 15-30 s). The
  held-out circle (10 mm, 0.15 Hz) is reach-limited: every predictive controller ties at 0.92 mm
  with 19 % saturation.
- Shared figure style `scripts/plot_style.py` (Okabe-Ito colours, a fixed colour + line style +
  marker per controller, at most 4-5 series per panel, 12 in wide, fonts 16/14/12/12, 150 dpi;
  heat maps cividis with values in cells). Restyled and regenerated `slide_task_*.png`,
  `slide_tasks_rmse.png`, `sweep_preview_gain.png`; their JSON numbers unchanged (diffed, timing
  only). `benchmark_slide_tasks.build_tasks` and `benchmark_all_tasks.recorded_runs` extracted
  from their `main()` so other scripts can import the references; both scripts' JSON output
  unchanged (rerun and diffed).
- Merged `mpc-controller` (frequency-domain identification `scripts/sysid_frf.py`,
  delay-compensated MPC `scripts/mpc_controller.py`, motion tests `scripts/motion_tests.py`,
  from the Data Driven Approach session) into `main`, after `docs/discussion-guide`.
  Checked after the merge: `compare_controllers.py` output identical; `mpc_controller.py`
  still 1.220 mm on the scored circle. Two test sets now coexist and are not yet unified:
  `motion_tests.py` (7 mm, 0.1 Hz circle/triangle/axial) and `benchmark_slide_tasks.py`
  (brief timing 0.4-0.5 Hz, scaled) + `benchmark_all_tasks.py` (recorded robot runs).
- Gain sweep (`scripts/sweep_preview_gain.py`, 70 runs, every run's parameters and metrics in
  `docs/figures/sweep_preview_gain.json`): lowering λ does not remove the delay-aware SMC's
  overshoot with the Taylor preview (circle 4.3 mm size 148-157 % for λ 0.5-8), because λ sets
  how fast σ decays, not its steady state y(t) = r~(t - T). With the exact preview the gain is a
  trade-off: smooth motions best at λ 0.5-1 (circle 0.28 mm, 103 %; axial 0.16 mm, 102 %),
  triangle best at λ 8 (0.30 mm). Paper SMC: lower λ shrinks its overshoot (183 -> 134 %) but
  error stays 6.4-9.0 mm. Figures now show the exact-preview controller's commands.
- Shape vs timing on the brief's motions (`scripts/benchmark_slide_tasks.py`, now reports
  shape = RMS distance to the reference path ignoring time, and size %). PD and Nikoletta's
  SMC trace the right shape but 0.65-0.9 s late on the 0.4 Hz circle (shape 0.35 / 0.65 mm,
  RMSE 7.48 / 5.71 mm); the delay-aware and paper SMCs are on time but 25-110 % too large.
- Cause of the delay-aware overshoot, tested: the Taylor reference preview r + T r' + T^2 r''/2
  (x1.27 on a 0.4 Hz circle; overshoots triangle corners by about edge speed x T). New option
  `DelayAwareNDOSMC(exact_preview=True)` takes r(t + T) directly: 4.3 mm circle 2.35 -> 0.54 mm
  (size 150 -> 112 %), triangle 1.99 -> 0.30 mm (128 -> 103 %), axial 2.73 -> 0.62 mm (after
  the first lap). Needs the trajectory in advance; the harness passes r(t) only. Default
  unchanged (`compare_controllers.py` output identical). Remaining 12 % on the circle:
  hypothesis = M2 underestimates 0.4 Hz reach by ~9 %. Paper-SMC overshoot: hypothesis =
  high-gain feedback through the unmodelled 1.1 s delay; not tested.
- `motion_tests.py` also reports the shape error: the distance from each tip sample to the
  nearest point of the reference path, ignoring timing. After the first 2 s (RMS):
  circle MPC 0.046 mm (SMC 1.63, PD 3.33); triangle 0.48 (0.98, 1.69), where the MPC's
  first lap before period detection dominates (0.042 with the true future reference).
  For axial motion the path is a line, so the shape error only measures off-axis
  deviation (0.000–0.017 mm for every controller) and says nothing about timing.

## 2026-10-26

- `scripts/benchmark_slide_tasks.py`: every controller of `compare_controllers.py` (same gains)
  on the brief's slide motions, `make_calibrated_sim(seed=0)`, 5 s settle at 4.5 psi, 30 s.
  Chosen: axial = open-loop path of S4 = 3.6 + 3.6 sin(2π·0.5 t) psi (slide command) with S2/S3
  at 4.5 psi (the harness baseline the models are linearised about; 1.5 psi, the bottom of the
  identification range, only adds a static offset and was run open-loop for comparison);
  circle_ol = open-loop path of S2-S4 = 4.5 ± 3.0 psi (1.5-7.5, the identified range), 0.4 Hz,
  phases 90/180/270° - a 2:1 ellipse (x 6.5 mm, y 14 mm peak-to-peak) because S1 is sealed and x
  has only S3; geometric circles at 0.4 Hz with r = 8.9 mm (M2 static limit: 4.5 psi × |H_x,S3| ×
  actuator gain = 8.94 mm) and r = 4.3 mm (M2 limit at 0.4 Hz with mechanics + 0.6 s lag, 4.37 mm);
  triangle R = 6.0 mm (static vertex limit 8.94 mm; quasi-static edge demand at 7.5 s/lap limits
  V0 to 6.02 mm). Reach checks (open loop): S3 at 0 / 9 psi holds +9.17 / -8.97 mm x (M2 8.94);
  S3 ±4.5 psi sine at 0.4 Hz gives 4.78 mm x amplitude (M2 4.37). Planar RMSE mm, axial /
  circle_ol / circle 8.9 / circle 4.3 / triangle: PD corrected 4.52 / 8.78 / 14.44 / 7.35 / 2.88;
  notebook SMC λ=8 4.08 / 6.32 / 9.95 / 5.67 / 2.06; paper SMC 5.61 / 9.91 / 11.42 / 8.07 / 2.05;
  delay-aware SMC 2.67 / 3.49 / 2.70 / 2.46 / 2.08; delay-aware NDOSMC 2.47 / 3.27 / 2.84 / 2.29 /
  2.06 (others in `docs/figures/benchmark_slide_tasks.json`). The 0.4-0.5 Hz tasks are where the
  delay-aware controllers pull ahead (they saturate 21-81 % of commands); on the slow triangle all
  model-based SMCs and notebook λ=8 tie at ~2.05 mm, set by the corners. A tracker that sees the
  reference through the 0.5 s delay + 0.6 s lag would score 3.42 / 7.15 / 12.44 / 6.01 / 3.61;
  it is not a lower bound (controllers extrapolating the periodic reference beat it). Open-loop
  replay of the generating commands scores exactly 0 (tip deterministic; the seed only drives
  pressure-sensor noise). Figures `docs/figures/slide_task_*.png`, `slide_tasks_rmse.png`.
- Project brief (RAS556 Course Project Intro PDF) defines the axial / circular / triangular
  test motions by open-loop slide demos (0.5 Hz, ~0.4 Hz, 7.5 s period; cm-scale, old τ ≈ 120 ms
  sim). Recorded in `docs/assignment.md`. Our benchmark used the recorded 0.1 Hz real-robot
  runs instead; the slide motions are now benchmarked too (entry above).
- `scripts/benchmark_all_tasks.py`: every controller of `compare_controllers.py` (same gains)
  tracking the lab's recorded real-robot tip trajectories (upstream `main`
  `data/_data_extract/parallel/{axial,circular,triangular}_2-5_parallel.csv`, read with
  `git show`, not copied), on `make_calibrated_sim(seed=0)`, harness-style (5 s settle at
  4.5 psi, then 180 s). Parallel / 2 psi charge matches `make_calibrated_sim()`'s defaults;
  5 psi peaks because the 10 psi runs command 10 psi. Mocap frame: RB3 - RB2 in RB1's frame,
  Y-up to Z-up, one yaw (84.8 deg) fitted against an open-loop replay; reference smoothed
  (1 Hz zero-phase). Planar RMSE mm axial / circular / triangular: PD corrected 3.88 / 4.79 /
  5.93; notebook SMC λ=8 2.63 / 3.00 / 4.17; paper SMC 1.66 / 2.61 / 2.73; paper NDOSMC 2.49 /
  3.58 / 3.40; delay-aware SMC 1.14 / 1.56 / 1.23; delay-aware NDOSMC 1.07 / 1.63 / 1.31
  (model-based ones saturate 20-59 % of commands). Open-loop replay of the recorded commands
  misses the recorded tip by 3.65 / 2.95 / 4.59 mm xy and 4.51 / 4.14 / 2.98 mm z: the real
  robot moves about twice as far along x (S3 vs sealed S1) as the sim, and its z includes a
  slow drift. "Axial" drives S3 only (0-5 psi): mostly x motion, not z. PD and notebook SMC
  do not control z. Gains were not retuned on these tasks.
- Gains tuned on the scored circle carry over: the delay-aware controllers stay best on all
  three recorded waveforms. `compare_controllers.py` rerun: identical to
  `controller_comparison.json` (max difference 0).
- Motion tests (`scripts/motion_tests.py`) for the course slides' triangular and axial
  motions. The slides define axial as a one-axis horizontal swing (one segment driven,
  the opposite compresses), not extension along z. MPC lateral RMSE: triangle 1.51 mm
  (SMC 2.03, PD 3.06); axial y 0.48 (1.42, 2.78); axial x 0.51 (1.46, 2.83).
- The AR reference preview could not foresee the triangle's corners (2.15 mm, worse
  than SMC). A periodic preview, which detects the reference's period from its own
  history and replays it, fixed it. It is now the default; the circle score is
  unchanged at 1.220 mm.

## 2026-10-21

- Lateral stiffness does not change with inflation on `make_calibrated_sim`
  (`scripts/stiffness_vs_pressure.py`, data only: tip + 0.1 mm noise, pressure sensors).
  Mean pressure 2.0-7.5 psi on S2-S4, ±0.5 and ±1.0 psi y-bend steps: gain from measured
  pressure to tip 1.798-1.811 mm/psi (±0.4 %), rate a 4.63-4.88 1/s (no trend, fit noise),
  identical for both step sizes (linear). Closes the gap left by the paper-style set-point
  test (which fitted command-to-tip, dominated by the delay and lag). The paper's
  pressure-dependent stiffness is therefore not present in the simulator; open for hardware.
- Extended to all axes and bent operating points (`scripts/stiffness_vs_pressure.py`, M2
  mechanics fit, 0.5 psi steps): x gain -1.81 to -1.84 mm/psi, rate 4.0-4.6 1/s; y gain
  1.80-1.82, rate 4.35-4.8; both flat over mean 2-7.5 psi and bend offset ±2 psi (no
  trend beyond fit noise). z gain falls 7 % (-1.47 to -1.36 mm/psi) from 2 to 7.5 psi mean;
  z rate not identifiable (13-38 1/s, hidden by the lag). Constant Λ, H justified for x, y.
- Decision (Ibraheem): `make_calibrated_sim` is the target plant; the benchmark is the
  package harness (7 mm circle, 0.1 Hz, seed 0). Recorded in AGENTS.md.
- Delay-compensated MPC (`scripts/mpc_controller.py`, branch `mpc-controller`): Kalman
  filter, delay predictor and unconstrained MPC on the identified models, with an AR(2)
  reference predictor. Scored 1.22 mm, against best SMC 2.34 and PD 4.08; 0.059 mm
  after the first 2 s. The predictor's first version used a 2 s window and a
  3×-range guard that rejected it in 959 of 3000 steps (2.13 mm); a 4 s window with an
  AR-pole stability guard fixed it. Details in `modeling-plan.md`.

- Frequency-domain identification finished (steps 5–7, PR #1): per axis
  γe^{−sT}/(τs + 1) × stage B fitted by Sanathanan–Koerner + weighted nonlinear least
  squares, realized in controllable canonical form behind the lag state. `make_sim`:
  4 states per axis, free-run fit 99.60 / 99.24 % (x / y) on a held-out random-step
  input; answer key 99.97 %. `make_calibrated_sim`: delay identified as 0.50 s,
  τ 0.5995 s, regulator gains 1.097 (S3) and 1.142 (S2/S4); x (S3 vs sealed S1) 3 states,
  95.60 %; y 4 states, 99.44 %. Table in `modeling-plan.md`.
- Delay fit needs a grid-searched start: with the 0.5 s delay the linear start (no
  delay) lands in a wrong local minimum on one channel.
- AGENTS.md: agents may switch the active `gh` account (and switch back after).

## 2026-10-20

- Frequency-domain identification (D3), step 4 and lag, `scripts/sysid_frf.py`
  (branch `sysid-frf`, PR #1). Excitation v2 (f0 0.01 Hz, 36 quasi-log odd lines to
  2.71 Hz, 10 below the 0.27 Hz lag corner vs 6 in v1) raised the median tip SNR from
  34.9 to 48.1 dB; command→tip FRF error vs the 50-state answer key fell from 2.6 %
  to 0.50 % median (max 64 % → 9.7 %). The FRFs (G_p(k) = Y_p(k) U_p(k)⁻¹ per period,
  averaged) match the answer key in all three stages; cross-coupling is at the noise
  level; |G| at 1 psi / 2 psi = 1.0003 (linear).
- The lag can be identified from data without assuming τ. From the pressure sensors
  (stage A, fit g e^(−sT)/(τs + 1)): τ = 0.5987 ± 0.0003 s, gain 0.994 (curvature
  coupling), T = −5.4 ms (half-sample alignment artefact, not a physical delay). A
  linear (Levy) least-squares fit without the delay term is biased to τ ≈ 0.577 s.
  From the tip alone (lag × one 2nd-order mode): τ = 0.645 ± 0.002 s, 7.5 % high,
  because the chain's slow real poles get lumped into the lag pole; pressure sensors
  are needed to separate them. The simulator integrates the lag by forward Euler
  (τ_eff = 0.5995 s per control step), recorded in `simulator.md`.

## 2026-10-14

- `notebooks/SMC_NDOSMC_delay_aware.ipynb`: Nikoletta's notebook (Sections 1-11
  unchanged) plus Sections 12-17 that run in Colab with only `soft-robotic-arm`
  installed: corrected allocator, the identified task-space model as an embedded
  parameter dict (from `docs/figures/task_space_model.json`, LTI), and the paper SMC,
  NDOSMC and delay-aware NDOSMC. Run headlessly (video cells stubbed): it reproduces
  9.94 / 4.08 / 4.37 / 3.20 / 1.46 / 2.84 / 1.21 mm, identical to the scripts.
- `scripts/render_runs.py` renders the scored run of the four best controllers as a
  2x2 video (3-D view + top-down tip trace) into `videos/` (git-ignored); the rendered
  runs reproduce the harness RMSEs exactly.

## 2026-10-12

- Qiao/Tao/Zhang SMC (eq. 5) and NDOSMC (eqs. 6-8, 12, 14) implemented in task space
  on the identified M1, plus a delay-aware variant on M2 (Smith predictor with the
  identified 0.5 s dead time, lag inversion, reference extrapolated by T):
  `scripts/ndosmc.py`. Paper's printed signs corrected: B = +1/c, and -d_hat in eq. 7.
  Scored task (`compare_controllers.py`, seed 0), whole-run / after-5-s RMSE mm: PD
  corrected allocator 4.08 / 3.96; notebook SMC best 2.45 / 2.13; paper SMC 1.46 / 0.87;
  paper NDOSMC 2.84 / 2.51 (observer saturates commands 31 % of samples); delay-aware
  without observer 1.23 / 0.09; delay-aware NDOSMC 1.21 / 0.10. Held-out 10 mm 0.15 Hz
  seed 5: 6.68, 4.89, 3.45, 3.19, 2.00, 1.97 mm, with 21-53 % saturation for the
  model-based ones. Modelling the actuator is the decisive step; the paper's NDOB adds
  ~0.02 mm once the delay is modelled. Gains were swept on the scored seed; every run is
  printed by `ndosmc.py --sweep`. Report: `docs/reports/controller-comparison.html`.
- Task-space model of `make_calibrated_sim` identified from sensors only (tip + 0.1 mm
  noise, pressure sensors; no `q`, `p_actual`, cfg or model): `scripts/task_space_model.py`,
  report `docs/reports/task-space-model.html`, parameters `docs/figures/task_space_model.json`.
  Actuator per segment: dead time 0.50 s, one lag 0.60 s, gains 1.120/1.096/1.164, biases
  0.45/0.15/0.30 psi; 0 psi command vents. Mechanics: second order, ~1.2 Hz, zeta ~0.75
  (well damped, no resonant peak); z dynamics too fast to identify. z lift kappa =
  2.5e-3 /mm. Paper-style M1 (Qiao 2024, command in, no delay) LPV parameters vary
  0.1 % / 0.0 % / 1.6-6.5 % over rho, so LTI suffices; free-run RMSE M1 0.57-1.32 mm in x/y
  (0.22-0.36 z) vs M3 (actuator + 2nd-order mechanics) 0.10-0.16 mm (noise floor 0.10). Residual x
  error is the S1 reservoir (leaks ~0.04 psi/min). The paper's printed B = -diag(1/c) has
  the wrong sign; its eqs. 5/13 need +1/c.
- Frequency-domain identification (D3), steps 0–3, `scripts/sysid_frf.py` (branch
  `sysid-frf`). Two differential channels (S1−S3, S2−S4, all levels) on `make_sim` at
  4.5 psi: static tip gain 1.8102 mm/psi per channel, cross gain 0.0000, ±2 psi
  responses exactly antisymmetric; matches the 50-state answer key. Lag alone gives
  t63 0.60 s; tip t63 0.81 s. Pressure sensors read the differential pressure 0.5 %
  low because the curvature offsets of opposing columns add (−0.0099 psi on 2 psi).
  Odd random-phase multisine (f0 0.02 Hz, 57 lines to 2.98 Hz, crest factor 2.46,
  ±2 psi): noise-free tip is periodic after one period (change 1e-16); no even-order
  distortion; odd-order distortion −105.5 dB at 2 psi, −117.5 dB at 1 psi (12 dB per
  halving, i.e. cubic). With 0.1 mm tip noise the detection lines sit at −24 dB (noise
  floor), median SNR 34.9 dB. Open: only 6 excited lines lie below the 0.27 Hz lag
  corner; a longer period or log-spaced lines would sample the slow dynamics better.
- Ported Nikoletta's SMC notebook to `scripts/smc_reduced_model.py`; it reproduces
  her table exactly on `make_calibrated_sim` + `CircularTrackingTask` (PD 500/20:
  9.94 mm; SMC λ=5, K=0.2, φ=0.02: 4.37 mm). Her one-mode model uses joint springs
  only; the hanging chain's gravity stiffness (0.83→0.09 N·m/rad from mount to tip,
  vs 0.42 spring) makes the true static gain 1.80 mm/psi, not 7.74. With gravity
  added and her gains, SMC gets worse (10.63 mm), so the result rests on the
  robust feedback, not model accuracy; the lag and 0.5 s delay are unmodelled.
  Her controller uses absolute tip position while the settled tip is at x −4.96 mm;
  using the deviation from the settled tip gives 4.20 mm (λ=5) and 3.70 mm (λ=8).
- The notebook's segment allocator (shared by her PD and SMC) uses each column's
  torque axis (−sin φ, cos φ) as the tip direction, rotating x and y by 90°: +1 psi
  of Δp_x moved the tip −2.07 mm in y (static test, `make_calibrated_sim` seed 0).
  Allocating along the measured tip directions (S2 +y, S3 −x, S4 −y) and flipping
  b's sign (`--fix-allocator`): PD 500/20 9.94 → 4.08 mm; SMC λ=5 4.37 → 3.20 mm,
  λ=8 2.53 mm; with `--deviation` 3.13 / 2.45 mm. Phase lags turn positive, which
  explains the negative lags. With gravity in the model it is still worse (11.16 mm).
  Every "max error" is 7.00 mm because the circle starts 7 mm from the settled tip.
  Step-by-step derivation page: `docs/reports/smc-model-derivation.html`.
- Read Qiao, Tao & Zhang 2024 in full (the paper Nikoletta's SMC follows): first-
  order LPV in configuration space q = [θ, L] per segment, gravity/inertia lumped into
  a disturbance estimated by an NDOB, σ = e, adaptive η, input torque/force with an
  inner pressure loop assumed fast (Fig. 2). Updated `references.md` (licence CC
  BY-NC-ND, pressure dynamics not modelled: both now verified) and `modeling-plan.md`.
  Her notebook departs from it: second order, tip space, constant K, no observer, so
  the omitted gravity is uncompensated. Added a comparison section to
  `docs/reports/smc-model-derivation.html`.
- Found the package's evaluation harness (`soft_robotic_arm.evaluation`, defaults
  to `make_calibrated_sim`, 3 inputs). Documented in `simulator.md`; which
  benchmark is graded is open.
- Forward kinematics of the sim checked in closed form against MuJoCo (max error
  1e-15 over 200 random poses); static pressure→tip map is 2×20, rank 2, x/y
  decoupled, 0.562 (P1) to 0.241 (P5) mm/psi; min-norm IK reached targets within
  0.001 mm in the sim. Scratch only, not yet a script.

## 2026-10-07

- Literature review of modelling / identification methods for the report (paper
  notes summarized there). Corrections: Qiao 2024 used a two-segment arm and its
  pressure-loop assumption is unverified; Papageorgiou 2024 is cable/McKibben-driven.
  Real-arm configuration conflict (3 driven segments + sealed column vs 20 pouches)
  added to `hardware.md` open questions. Lab's real data: 36 runs at 0.1 Hz in
  upstream `main` `data/_data_extract/`, used for the sim calibration (bounded
  differential evolution; 3.0 mm lateral RMSE mechanics replay).

- Built the ground-truth linear plant model (`scripts/plant_model.py`, 50 states,
  continuous time, about 4.5 psi). Matches the nonlinear sim to 0.1 % over the
  full 0–9 psi range. Controllable 50/50; observable 16/50 from tip x, y, 40/50
  with pressure sensors. Hankel singular values suggest about 4–5 states per axis.
  Details in `modeling-plan.md`. Added `scipy` as a dependency.

- `observe()` also returns privileged `p_actual` and full `q`, and tip sensors are
  noise-free (read in `simulator.py`, `c92931d`). Identification should use only
  hardware-available signals plus added tip noise; see `simulator.md`.

- Moved all project context into the repo (README, AGENTS.md, `docs/`). The
  coursework notebook is now in `notebooks/`, with outputs and Colab metadata
  stripped on commit by an nbstripout git filter (`scripts/setup_git.sh`).
- Reproduced the professor's PD baseline locally: **4.10 mm** RMSE
  (`scripts/run_baseline.py`), same as the Colab output.
- Lab slides received (see `docs/hardware.md`): the real robot has one SMC ITV1050
  regulator per pouch, so 20 independent pressure inputs exist on hardware too.
- Slides say pneumatic τ ≈ 120 ms; the simulator uses 0.6 s (changed upstream on
  2026-09-30). Use 0.6 s.
- Confirmed P1 is at the mount (drives `level0`) in the hanging sim; the notebook's
  "bottom" refers to the upright drawing. Slider viewer rows reordered to match.
- Added `scripts/draw_kinematics.py` and `docs/figures/kinematic_diagram.*`.
- Replaced the viewer's keyboard control with a Tk slider panel (MuJoCo's viewer
  already binds 0–5, Q, W, E, R …). Fixed a hang on close (waits for the render
  thread before interpreter shutdown).

## 2026-10-05 / 06

- Created the repo and invited Nikoletta (write access).
- Read the simulator source: τ = 0.6 s lag, linear pressure→force, 15-DOF linear
  spring–damper chain, curvature-coupled pressure sensors. `make_sim()` has no
  transport delay (that belongs to `make_calibrated_sim()`).
- PyPI 0.4.2 is byte-identical to upstream `simulation@c92931d`.
- Discussed modelling options and RL; plan in `docs/modeling-plan.md`.

## Open questions

- Which τ reflects the real regulators, 0.6 s or 120 ms? (Ask the professor or
  Jeevan Hebbal Manjunath.)
- Is RL acceptable as an extension beyond the proposal scope?
- Which reference trajectories and metrics will the final comparison use beyond
  the notebook's random reference? (Rubric, deliverables and due dates not yet
  recorded here.)
- How the work is split between Nikoletta and Ibraheem (not yet recorded).
