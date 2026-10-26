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
5. **Ground truth:** the linearized plant below (`scripts/plant_model.py`). Use it
   only to check identified models; on hardware it would not exist, and the course
   likely expects identification from input/output data.

## Ground-truth linear plant model (done)

`uv run python scripts/plant_model.py` builds the open-loop plant (no controller)
as a continuous-time state-space model ẋ = Ax + Bu, y = Cx about every pouch at
4.5 psi. State x = [qpos 15, qvel 15, p_actual 20], n = 50; u = 20 pouch commands;
y = tip x, y, z and the 20 pressure sensors. Mechanics rows of A are central finite
differences of MuJoCo's `mj_forward` acceleration (with the sim's own
pressure→force map); the lag rows are −I/τ and I/τ exactly. Numbers below are from
that run (soft-robotic-arm 0.4.2, 2026-10-07).

- **Stable**: slowest pole −1.25 /s; lag pole −1/τ = −1.667 /s (×20); fastest
  −15,500 /s.
- **Matches the nonlinear sim in open loop** (figure
  [`figures/plant_model_validation.png`](figures/plant_model_validation.png)): tip
  x, y error RMS 0.004 mm against 4.33 mm signal RMS (0.1 %) for opposing columns
  driven over the full 0–9 psi range (peak tip deflection ≈ 7.5 mm). Over this
  range the plant is effectively linear.
- **Controllable**: 50/50 from the 20 pouch commands.
- **Observable** (PBH test per eigenvalue):
  - 16/50 from tip x, y alone. 18 of the 20 identical lag modes can't be seen with
    2 outputs (a mode of multiplicity g needs g outputs).
  - 25/50 from tip x, y, z.
  - 40/50 from tip x, y plus the 20 pressure sensors. All lag states become
    observable. The 10 still unobservable are the modes that become observable once
    tip z is added, i.e. the axial (extension) dynamics, which don't move x, y at
    the straight equilibrium.
- **Don't use rank [B AB … A⁴⁹B]** here: with poles spanning −1 to −15,000 /s it is
  numerically meaningless (it gave 8, 16 and 35 for the same model, depending on
  the method; at 100 Hz the stiff modes become discrete eigenvalues ≈ 1e-12). Use
  the PBH test, Gramians or a staircase form.
- **Hankel singular values**, 20 pouches → tip x, y, normalized; they come in x/y
  pairs: 1, 0.235, 0.091, 0.036, 3.6e-3, 2.3e-4, then < 1e-5. So about 4–5 states
  per axis capture the input–output behaviour; identified models should land near
  that order.

## Identified model, frequency domain (done)

`scripts/sysid_frf.py` (branch `sysid-frf`), from hardware-available signals only.
Per axis, input = differential channel command u [psi], output = tip position [m]:

```
G(s) = γ e^{−sT} / (τs + 1)  ×  (b₁s + b₀) / (s³ + a₂s² + a₁s + a₀)
       stage A: lag (from pressure sensors)   stage B: mechanics (pressure -> tip)
```

| | `make_sim`, x | `make_sim`, y | `make_calibrated_sim`, x | `make_calibrated_sim`, y |
|---|---|---|---|---|
| τ [s] | 0.5987 | 0.5986 | 0.5995 | 0.5994 |
| γ | 0.994 | 0.994 | 1.097 | 1.142 |
| T [s] | 0 | 0 | 0.50 | 0.50 |
| stage B order | 3 | 3 | 2 | 3 |
| stage B poles [1/s] | −4.15 ± 5.56j, −1.60 | −4.01 ± 5.56j, −1.68 | −5.15 ± 5.43j | −4.15 ± 5.57j, −1.55 |
| free-run fit, held-out input | 99.60 % | 99.24 % | 95.60 % | 99.44 % |

- Order chosen per axis by free-run error on a held-out random-step input (seed 101),
  lowest order within 2 % of the best; unstable fits rejected.
- `make_sim`: 4 states per axis, in line with the Hankel singular values. Answer key
  (50 states) reaches 99.97 % on the same input.
- `make_calibrated_sim` x is driven by S3 alone against the sealed S1 reservoir; higher
  orders add a slow pole (≈ −0.05 1/s) that free-runs worse. Its detection lines sit at
  −48 dB (vs −109 dB on `make_sim`) and the response drifts slowly between periods,
  so that axis is not exactly LTI.
- The state-space realization (lag state, then stage B in controllable canonical form)
  is printed by `uv run python scripts/sysid_frf.py fit [--plant calibrated]`.

## Delay-compensated MPC on `make_calibrated_sim` (done)

`scripts/mpc_controller.py`, built on `models/sysid_calibrated_v2.json` (written by
`sysid_frf.py fit --plant calibrated`). Per axis:
- a Kalman filter on [model state; tip offset], measuring tip and channel pressure;
- a delay predictor, x_{n+d} = A_d^d x_n + Σ A_d^{d−1−i} B_d u_{n−d+i};
- unconstrained MPC over 3 s on Δu-penalized tracking error (ρ = 1e-5, tuned on a
  5 mm, 0.07 Hz task with seed 5);
- reference preview from an AR(2)-plus-intercept predictor on the reference's own
  history.

| Controller (harness, seed 0, 7 mm at 0.1 Hz) | RMSE | RMSE after 2 s |
|---|---|---|
| PD kp 500, kd 20 (corrected allocator) | 4.08 mm | 3.96 mm |
| SMC λ 5, K 0.35, φ 0.02, deviation (corrected) | 2.34 mm | 2.06 mm |
| MPC, no preview | 3.64 mm | — |
| **MPC, predicted reference** | **1.22 mm** | **0.059 mm** |
| MPC, true future reference (diagnostic) | 1.16 mm | — |

- The RMSE that remains is the start-up transient: the reference jumps 7 mm at t = 0,
  and nothing acts for 0.5 s. The diagnostic row shows that the predictor costs 5 %.
- Held-out cases: 4 mm radius 0.68 mm (SMC 1.42), 10 mm 1.88 (3.32), 0.05 Hz 1.21
  (2.19), 0.2 Hz 1.29 (3.01). With 0.1 mm or 1 mm tip noise added: 1.220 and 1.235 mm.
- The S3 command saturates at 9 psi only during the start-up transient, then swings
  0.6–8.3 psi, so circles much larger than 10 mm will saturate the x channel.

### Other reference motions (`scripts/motion_tests.py`)

Definitions follow the RAS 556 project-introduction slides ("Simulation Results
Circular / Axial / Triangular Motion"):
- **Triangular:** straight-line interpolation through V0 (+x), V1 (upper left) and V2
  (lower left).
- **Axial:** the tip swings along one horizontal axis; the slides drive one segment
  (Seg 4) and the opposite one compresses. That is not extension along z.

Here the triangle is equilateral with circumradius 7 mm, 10 s per lap; axial is
7 mm·sin(2π·0.1 t) along y or along x, with the other axis held. Settle, duration and
seed are as in the harness; the loop reproduces `evaluate_controller` exactly.

| Lateral RMSE [mm] | MPC | MPC, AR preview only | SMC | PD |
|---|---|---|---|---|
| circle | 1.220 | 1.220 | 2.342 | 4.080 |
| triangle | **1.510** | 2.149 | 2.027 | 3.055 |
| axial along y | **0.480** | 0.480 | 1.421 | 2.776 |
| axial along x | **0.511** | 0.511 | 1.460 | 2.826 |

- **Triangle.** The AR preview cannot foresee corners, which made the MPC no better
  than SMC (2.15 vs 2.03 mm).
  - The fix: the reference predictor detects a repeating period in the reference's
    history and replays it. It is exact for any periodic reference once one lap has
    been seen (detection needs P + 2 s, about 12 s here). It is now the default and
    leaves the circle and axial results unchanged.
  - During the first lap S3 saturates at 0 and 9 psi near the corners. After
    detection the error is 0.05–0.3 mm, with spikes only at the corners.
  - With the true future reference the MPC would reach 0.957 mm.
- **Axial.** The held axis stays within 0.005 mm. The z by-product is +0.17 mm along
  y, and −1.6 to +1.9 mm along x (S3 alone bends and extends the arm).

## Consequences for SMC

- Relative degree 3 breaks the textbook surface s = ė + λe (which assumes the input
  appears in ë). Two standard fixes:
  - **Cascade (preferred).** SMC on the mechanical part with pressure as a virtual
    input, plus an inner loop that drives measured pouch pressure to the SMC's
    demand (lag inverse + feedback). Qiao et al. 2024 use this structure (their Fig. 2) but assume the inner loop is
    fast and do not model it; here the inner loop must be built deliberately.
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
