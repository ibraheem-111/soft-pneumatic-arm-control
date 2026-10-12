# Project log

Newest first. Record decisions, findings and open questions here so the next
person (or agent) can pick up without asking. Keep entries short and dated.

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
