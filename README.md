# soft-pneumatic-arm-control

RAS556 course project (Fall 2026) by **Nikoletta Biri** and **Muhammad Ibraheem**:
system identification and controller design for a four-segment, twenty-pouch
pneumatic soft robotic arm, simulated in MuJoCo with the
[`soft-robotic-arm`](https://pypi.org/project/soft-robotic-arm/) package from Wenlong
Zhang's lab at ASU ([source](https://github.com/Jeevan-HM/Soft-Robotic-Arm/tree/simulation/mujoco)).

**Goal (from our [proposal](docs/proposal/proposal.md)):** understand how pressure
inputs move the arm, build a linear feedback controller, then explore sliding mode
control, and compare controllers on several reference trajectories for tracking
accuracy and robustness. **The number to beat** is the professor's PD baseline:
**4.10 mm** planar RMSE on the notebook's random trajectory.

![Kinematic diagram](docs/figures/kinematic_diagram.png)

## Quick start

Needs [uv](https://docs.astral.sh/uv/) and Python 3.12.

```bash
git clone https://github.com/ibraheem-111/soft-pneumatic-arm-control.git
cd soft-pneumatic-arm-control
uv sync                     # installs soft-robotic-arm 0.4.2, MuJoCo, nbstripout
./scripts/setup_git.sh      # once per clone: notebook-stripping git filter (see below)
uv run python scripts/run_baseline.py   # prints "Planar tracking RMSE: 4.10 mm"
```

| Command | What it does |
|---|---|
| `uv run python scripts/run_baseline.py` | Runs the professor's PD controller (taken from the notebook) on the benchmark: sim seed 2, 20 s. |
| `uv run python scripts/view_arm.py` | MuJoCo viewer + slider window, one slider per pouch (rows P1 at the mount → P5, columns S1 +x, S2 +y, S3 −x, S4 −y; top slider sets a whole segment). |
| `uv run python scripts/view_arm.py --dump-xml soft_arm.xml` | Writes the generated MuJoCo model (MJCF). |
| `uv run python scripts/draw_kinematics.py` | Regenerates `docs/figures/kinematic_diagram.{svg,png,pdf}` from the model. |

## The coursework notebook

[`notebooks/soft_robotic_arm.ipynb`](notebooks/soft_robotic_arm.ipynb) is the
professor's notebook as we use it in Colab:
[**Open in Colab**](https://colab.research.google.com/github/ibraheem-111/soft-pneumatic-arm-control/blob/main/notebooks/soft_robotic_arm.ipynb).
It installs the package with `%pip`, shows per-pouch commands and pressure readings,
then runs the PD baseline and renders a video. The upstream original is in the
simulator repo
([GitHub](https://github.com/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/coursework/soft_robotic_arm_new.ipynb),
[Colab](https://colab.research.google.com/github/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/coursework/soft_robotic_arm_new.ipynb));
it also lists the student tasks, copied into [`docs/assignment.md`](docs/assignment.md).

**Notebook hygiene:** `.gitattributes` routes `*.ipynb` through an nbstripout
filter that removes outputs, execution counts and Colab/Jupyter run metadata on
`git add`, so diffs show only code and prose. Your local file keeps its outputs.
Git filters live in `.git/config`, which is not committed, so **every clone must run
`./scripts/setup_git.sh` once**. Without it, git silently skips the undefined filter
and a notebook would be committed with all its outputs. To save a notebook from Colab back into the repo, download it
(File → Download .ipynb) into `notebooks/` and commit; the filter cleans it.

## What we know about the plant (short version)

Details and evidence: [`docs/simulator.md`](docs/simulator.md).

- **Inputs:** a `(4, 5)` array of absolute pouch pressures in psi, kept in 0–9.
  Rows S1–S4 push the tip toward +x, +y, −x, −y. **P1 is next to the mount, P5
  nearest the tip** (the notebook's "bottom" means the upright drawing).
- **Outputs:** `tip_pos`, `tip_vel` (m, m/s), `pouch_pressures` (4, 5),
  `segment_pressures` (4,), `time`.
- **Dominant dynamic:** each pouch's pressure follows its command with a
  first-order lag, **τ = 0.6 s**. The lab slides say 120 ms; that is an older
  simulator version.
- **Pressure → force is linear;** pouch k acts only on level k. Bending at each level
  comes from the difference between opposing columns, stretching from their sum.
- **Mechanics:** a hanging chain of 5 levels, each with an axial slide and two bending
  hinges (15 DOF), linear springs and dampers. Nonlinearity comes only from geometry
  and gravity.
- **Sensors:** measured pressure = true pressure + bending-dependent offset + 0.002 psi
  noise.
- `make_sim()` has no transport delay and no sealed reservoir; those belong to
  `make_calibrated_sim()`, which the coursework does not use.
- The real robot has one regulator per pouch, so all 20 inputs exist on hardware
  too ([`docs/hardware.md`](docs/hardware.md)).

## Status and next steps

Done: repo setup, plant analysis, viewer, kinematic diagram, baseline reproduced.
No identification or new controller is implemented yet.

Next, per [`docs/modeling-plan.md`](docs/modeling-plan.md):

1. Identification experiments: pressure lag on its own, then a gray-box model and
   ARX/N4SID, with linearity checks; validate on multi-step simulation error.
2. Linear controller with reference feedforward that cancels the lag.
3. Cascade sliding mode control + disturbance observer; robustness tests.
4. Optional: Koopman/SINDYc if nonlinearity warrants it; residual RL (ask the
   professor first).

Decisions, findings and open questions are logged in [`docs/log.md`](docs/log.md).

## Repository layout

```
README.md                 this file
AGENTS.md                 working rules for collaborators and AI agents (CLAUDE.md imports it)
docs/
  assignment.md           course, notebook, student tasks, controller contract, baseline
  simulator.md            how make_sim() works, verified against source
  hardware.md             the physical robot (from the lab's slides)
  modeling-plan.md        identification techniques, SMC implications, RL, plan
  references.md           papers (DOIs), code, notebooks, textbooks
  log.md                  dated decisions and open questions
  proposal/               our submitted proposal (.doc + markdown transcription)
  figures/                generated figures
notebooks/                coursework notebook (stripped on commit)
scripts/                  runnable tools (see table above) + setup_git.sh
main.py                   uv's placeholder entry point (unused)
```

## Collaborating

- Read [`AGENTS.md`](AGENTS.md) before contributing, whether you're a person or an
  AI agent (Claude Code reads it through `CLAUDE.md`; Codex reads `AGENTS.md`
  directly).
- Collaborators: [@ibraheem-111](https://github.com/ibraheem-111) (owner),
  [@Niki-89-AI](https://github.com/Niki-89-AI) (write).
- Upstream simulator: [Jeevan-HM/Soft-Robotic-Arm](https://github.com/Jeevan-HM/Soft-Robotic-Arm)
  (`simulation` branch for the MuJoCo package, `main` for the lab's hardware code).
