# Modelling and control plan

Working plan agreed so far (October 2026). Nothing in steps 1–4 below is
implemented yet. Plant facts it relies on are in [`simulator.md`](simulator.md);
reading for each technique is in [`references.md`](references.md).

## What a correct model has to capture

```
command u (psi) ──► first-order lag, τ = 0.6 s ──► pouch pressure p ──► linear moment/force
                                                                          │
tip x, y ◄── mildly nonlinear geometry ◄── 15-DOF spring–damper chain + gravity ◄┘
```

- **The lag** is the slowest and most important dynamic (discrete pole ≈ 0.983 at
  100 Hz).
- **The mechanics** are roughly second order per bending direction at the tip: 5
  levels, but one or two modes dominate.
- **The inputs:** 20 pouch commands collapse to opposing-column differences (x and y
  bending at each level) plus a shared pressure that mostly stretches the arm.
  Fitting all 20 raw inputs wastes data.
- So the path from pressure command to tip position is about **order 3 per axis**
  (1 lag + 2 mechanical), with **relative degree 3**.
- The pressure states are measured (`pouch_pressures`), so the lag can be identified
  on its own, directly from sensors.

## Candidate techniques (simplest first)

1. **Gray-box physical model (do first).** Fix the structure — first-order lag
   feeding a second-order mass–spring–damper per axis — and fit τ, gains, ωₙ, ζ from
   step and multisine tests. Interpretable, defensible in the report, maps straight
   onto SMC design.
2. **Linear black-box: ARX/ARMAX or subspace (N4SID).** The standard control-course
   route (`sippy`, MATLAB's System Identification Toolbox, or least squares for ARX).
   Choose the order by AIC or validation error; expect about 3 per axis. This is the
   "linear model" in the proposal and supports LQR/LQG, H∞ or linear MPC.
3. **SINDYc** (Brunton et al.; used by Papageorgiou 2024 for SMC). Sparse regression
   over candidate terms gives readable nonlinear equations. Needs derivatives:
   velocity comes from `tip_vel`, acceleration must be differentiated and smoothed.
   Worth it only if a real nonlinearity shows up (amplitude dependence, x–y
   coupling, saturation).
4. **Koopman / EDMD** (Bruder et al. 2021). Lift the state with basis functions plus
   time-delay embedding, fit a linear model in the lifted space, then do linear MPC
   (a QP). Bruder needed it because their arm was strongly nonlinear; here a plain
   delay-embedded linear model may do about as well — a worthwhile comparison
   either way.
5. **LPV** (Qiao, Tao, Zhang 2024). Parameters vary with an operating point such as
   the bias pressure. Only if gains change noticeably between, say, 2 and 7 psi bias.
6. **Neural networks (NARX/LSTM).** Accurate but opaque, and hard to design SMC on.
   Skip.

## How to run the identification

1. **Excitation.** PRBS or multisines on the differential inputs (S1−S3, S2−S4)
   around a 4.5 psi bias, staying inside 0–9 psi. Cover about 0.02–3 Hz so both the
   lag (corner ≈ 0.27 Hz) and the mechanical modes are excited. Repeat at several
   biases to test operating-point dependence.
2. **Check linearity before choosing a nonlinear method.** Compare step responses at
   ±1, ±2 and ±4 psi, and test superposition (x step + y step vs both together). If
   responses scale linearly, techniques 3–5 must justify themselves against 2.
3. **Validate on multi-step simulation error, not one-step prediction.** At 100 Hz
   almost any model has a great one-step fit. Run the model forward on held-out data
   (e.g. the notebook's random reference) and report RMSE in mm.
4. **Separate seeds and trajectories** for training and validation.
5. **Optional ground truth:** `mujoco.mjd_transitionFD` linearizes the true model.
   Use it only to check the identified model — on hardware it would not exist, and
   the course likely expects identification from input/output data.

## Consequences for SMC

- Relative degree 3 breaks the textbook surface s = ė + λe (which assumes the input
  appears in ë). Two standard fixes:
  - **Cascade (preferred).** SMC on the mechanical part with pressure as a virtual
    input, plus an inner loop that drives measured pouch pressure to the SMC's
    demand (lag inverse + feedback). This is the structure Qiao et al. assumed; here
    the inner loop must be built deliberately.
  - **Higher-order surface** s = ë + λ₁ė + λ₀e, with states estimated from the
    identified model.
- Add a **disturbance observer** (Qiao's NDOB, or an ESO) to absorb model error, so
  the switching gain stays small and chattering stays manageable.

## RL

Feasible: the simulator runs about 1,100 control steps/s per core (1 M steps ≈
15 min), so PPO/SAC with 8–16 parallel environments is practical on a laptop.

- **For:** handles saturation, nonlinearity and all 20 inputs with no model; a good
  "what's achievable" benchmark.
- **Against, for this project:** the plant is nearly linear with a known lag, so
  model-based control (LQR/MPC with reference feedforward) will likely match or
  beat RL for far less effort, and with guarantees. RL gives no stability or
  robustness proofs, while the proposal's contribution is a robustness comparison.
  Pitfalls: the observation needs pressures and a reference preview (because of the
  lag); train across random seeds and perturbed parameters (gains, τ) to avoid
  overfitting one plant; evaluate on unseen trajectories.
- **Best fit: residual RL** — a policy learns a small correction added to the
  model-based controller. Keeps structure and stability arguments, trains far
  faster than from scratch, and gives a clean ladder: baseline PD → model-based
  linear → SMC (+ observer) → residual RL.
- RL is outside the submitted proposal; check with the professor before investing.

## Suggested path

1. **Identification experiments:** the pressure lag on its own, then a gray-box
   model and ARX/N4SID, with the linearity checks above.
2. **Linear controller with feedforward:** invert the identified model along the
   reference to cancel the lag. Expected to cut the 4.10 mm baseline substantially —
   to be measured, not assumed.
3. **Cascade SMC + disturbance observer,** compared under disturbances and parameter
   changes (randomized τ and gains, added tip mass, more sensor noise).
4. **Optional:** Koopman/SINDYc if the linearity tests call for it; residual RL as a
   final comparison.
