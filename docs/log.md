# Project log

Newest first. Record decisions, findings and open questions here so the next
person (or agent) can pick up without asking. Keep entries short and dated.

## 2026-10-21

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

## 2026-10-12

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
