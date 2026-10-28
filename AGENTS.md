# AGENTS.md

Instructions for anyone working in this repo: Nikoletta, Ibraheem, and the AI
agents either of us uses (Claude Code reads this via `CLAUDE.md`; Codex reads it
directly). Read `README.md` first for the project overview.

## Context lives in the repo

Everything needed to continue the project is in `README.md` and `docs/`. Nothing
should depend on a chat session or on someone's memory.

- Before working, read `docs/log.md` (latest state, open questions) and whichever
  of `docs/assignment.md`, `docs/simulator.md`, `docs/modeling-plan.md` your task
  touches.
- When you learn something non-obvious (a plant fact, a decision, a failed
  approach, a number), add it to the relevant doc and a dated line to `docs/log.md`
  in the same change. Prefer editing existing docs over creating new ones.
- Cite where a fact comes from (source file, experiment, paper) and say how it was
  checked. Mark anything unverified as such.

## Environment

- Python 3.12 via uv. `uv sync` installs everything; run code with `uv run …`.
- Add dependencies with `uv add <pkg>` (or `uv add --dev <pkg>`), never with
  `pip install`, so `uv.lock` stays authoritative.
- The simulator is `soft-robotic-arm` 0.4.2 from PyPI. Do not vendor or patch it. If
  behaviour depends on its internals, read the upstream source at
  <https://github.com/Jeevan-HM/Soft-Robotic-Arm/tree/simulation/mujoco> (release
  commit `c92931d`) and record the finding in `docs/simulator.md`.
- `main.py` is uv's unused placeholder.

## Plant facts that are easy to get wrong

Full detail in `docs/simulator.md`.

- Pneumatic lag τ = **0.6 s** (not the slides' 120 ms). It dominates tracking.
- **Target plant: `make_calibrated_sim()`** (decision by Ibraheem, 2026-10-21): the
  three-input plant fitted to hardware data, with a sealed S1 reservoir and a **0.5 s
  transport delay** on top of the lag. `make_sim()` has no delay; keep it for reference.
- Commands are absolute psi kept within **0–9 psi**: `(3,)` for S2–S4 on
  `make_calibrated_sim()`, `(4, 5)` (or `(4,)`) on `make_sim()`. Rows S1–S4 push the
  tip toward +x, +y, −x, −y.
- **P1 is next to the mount** (`level0`), P5 nearest the tip, in the hanging sim.
- Pouch k affects only level k; pressure→force is linear; opposing columns cancel
  in bending.
- MuJoCo's passive viewer binds 0–5 and most letters; don't add keyboard controls
  to viewer scripts, use the Tk slider pattern in `scripts/view_arm.py`.

## Controllers and experiments

- Use the package harness's interface: `compute(t, obs, ref) -> finite (3,)` array of
  absolute psi for S2–S4. Controllers never call `sim.step()` and use only
  hardware-available signals (tip, pouch pressures), never `p_actual` or `q`.
- Benchmark with `soft_robotic_arm.evaluation.evaluate_controller` on
  `make_calibrated_sim(control_hz=100, seed=0)` and the default `CircularTrackingTask`
  (7 mm circle, 0.1 Hz, 30 s, RMSE over the whole run). Current numbers: PD
  **4.08 mm**, notebook SMC (corrected allocator) **2.34 mm**, delay-aware SMC on M2
  **1.23 mm** (`scripts/compare_controllers.py`), delay-compensated MPC **1.22 mm**
  (`scripts/mpc_controller.py`); the last two are effectively tied. Tune on a different
  task or seed, never the scored one. The old `make_sim` PD baseline (4.10 mm, `scripts/run_baseline.py`) is
  kept for reference.
- Report results as numbers you actually ran, with the command that produced them.
  Never estimate a result you did not measure.
- Fix seeds for anything reported. Use different seeds and trajectories for
  identification data and validation data.
- Validate identified models on multi-step (free-run) simulation error, not
  one-step prediction.

## Code style

- Scripts go in `scripts/`, runnable as `uv run python scripts/<name>.py`, with a
  module docstring that states purpose and usage.
- Match the existing style: plain NumPy/Matplotlib, type hints on public functions,
  comments only where intent is not obvious.
- Read parameters from the package (`sim.cfg`, the MuJoCo model) rather than
  hard-coding numbers that already exist there.
- Generated figures go in `docs/figures/` and must be reproducible by a script.
- Figures use `scripts/plot_style.py` (`P.apply()`, `P.line(name, n)`): Okabe-Ito colours,
  one fixed colour + line style + marker per controller, at most 4-5 series per panel,
  readable sizes. Add new controllers to `plot_style.SERIES`.

## Notebooks

- Notebooks live in `notebooks/`. Run `./scripts/setup_git.sh` once per clone; it
  configures the nbstripout filter that `.gitattributes` applies to `*.ipynb`, so
  outputs and Colab metadata never reach git.
- Git silently ignores the filter if it isn't configured, so before committing a
  notebook check `git config filter.nbstripout.clean` prints a command. If it prints
  nothing, run the setup script first. Never bypass the filter.
- Prefer putting reusable code in `scripts/` (or a package module) and keeping
  notebooks for exploration and presentation.

## Git and GitHub

- Default branch `main`. For anything beyond a small fix, work on a branch and open
  a PR so the other teammate can review.
- Don't commit large binaries (videos, datasets, PDFs of copyrighted papers). Link
  papers in `docs/references.md` instead.
- Don't force-push `main`, and don't rewrite shared history.
- `gh` may have several accounts logged in. Pushing to this repo needs an account
  with access: `ibraheem-111` (owner) or `Niki-89-AI` (collaborator). Check with
  `gh auth status`; switch with `gh auth switch --user <name>`. AI agents may switch
  the active account themselves when the active one lacks access (Ibraheem's
  machine often has `ibraheem-tenexity` active); switch back to the previously active
  account when done.
- AI agents: follow your harness's commit-attribution conventions, keep commit
  messages descriptive (what and why), and don't push unless the person you're
  working for asked you to.
