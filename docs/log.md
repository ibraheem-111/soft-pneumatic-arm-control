# Project log

Newest first. Record decisions, findings and open questions here so the next
person (or agent) can pick up without asking. Keep entries short and dated.

## 2026-10-07

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
