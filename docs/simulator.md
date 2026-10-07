# The simulator: what `make_sim()` actually is

Everything here was read from the package source and checked by running it, with
`soft-robotic-arm` 0.4.2 installed from PyPI. Upstream source:
[Jeevan-HM/Soft-Robotic-Arm, branch `simulation`, directory `mujoco/`](https://github.com/Jeevan-HM/Soft-Robotic-Arm/tree/simulation/mujoco).
The upstream docs worth reading are
[`docs/arm_model_guide.md`](https://github.com/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/docs/arm_model_guide.md)
and
[`docs/arm_parameters.md`](https://github.com/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/docs/arm_parameters.md);
fitted values live in
[`soft_robotic_arm/data/calibration.json`](https://github.com/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/soft_robotic_arm/data/calibration.json).

## Provenance

- PyPI: [`soft-robotic-arm`](https://pypi.org/project/soft-robotic-arm/), releases
  0.4.0, 0.4.1, 0.4.2 (all uploaded 2026-10-04). 0.4.2 is the latest.
- The installed 0.4.2 files (`__init__.py`, `calibration.py`, `evaluation.py`,
  `model.py`, `simulator.py`, `data/calibration.json`) are byte-identical to
  upstream commit `c92931d` ("build: release version 0.4.2") on branch
  `simulation`. The branch tip `ac24f27` only changes the README and notebook.
- Upstream branches: `main` (50 commits, to 2026-04-18) is the lab's hardware repo
  (ROS 2 `colcon_ws` with a motion-capture bridge, recorded experiments, MECC paper
  plots, open-loop controller). `simulation` (started 2026-08-28) is an orphan branch
  with no shared history; it holds the MuJoCo package and a separate SOFA FEM sim
  (`soft-arm-sofa/`). The calibration was fitted to 36 recorded robot runs, which
  probably come from `main`'s `data/`/`experiments/` — useful if we ever validate
  against hardware.
- To browse upstream locally: clone it and add a worktree for the branch, e.g.
  `git clone https://github.com/Jeevan-HM/Soft-Robotic-Arm.git && cd Soft-Robotic-Arm && git worktree add ../Soft-Robotic-Arm-simulation simulation`.

## `make_sim()` vs `make_calibrated_sim()`

The coursework uses `make_sim()`. It passes an explicit `ArmConfig`, so it does
**not** get the calibrated actuator path:

| | `make_sim()` (coursework) | `make_calibrated_sim()` |
|---|---|---|
| Inputs | all 4 segments, `(4,)` or `(4, 5)` | 3 values for S2–S4 |
| Segment 1 | normal actuator | sealed, charged pressure reservoir |
| Transport delay | none | 0.5 s |
| Regulator gain / bias | 1 / 0 | per-segment fitted gain and bias |
| Pneumatic lag τ | 0.6 s | 0.6 s |

## Signal path for one `sim.step(command)` (100 Hz)

```
command (4,5) psi, clipped 0..11 internally (coursework limit 9 psi)
  → first-order lag per pouch, τ = 0.6 s            (p_actual)
  → generalized forces written to data.qfrc_applied every 1 ms substep
  → 10 × 1 ms MuJoCo steps (implicitfast, gravity on, contacts disabled)
  → obs: tip_pos, tip_vel (site sensors), pouch_pressures = p_actual + sensor model
```

### Pneumatic lag — τ = 0.6 s, not 120 ms

- `cfg.tau_pneumatic = 0.6` in 0.4.2, and `calibration.json` says the same.
- Checked empirically: a step on one pouch reaches 63 % of its target at 0.60 s.
- The notebook agrees: a 2 psi command reads 1.62 psi after 1 s, and
  2·(1 − e^(−1/0.6)) = 1.62. With τ = 0.12 s it would read 2.00 psi.
- The lab slides (see [`hardware.md`](hardware.md)) say τ ≈ 120 ms. That was the
  original twin's value (commit `69117e3`, 2026-08-28); it changed to 0.6 s in
  `a654f09` (2026-09-30), around the recalibration. Use 0.6 s for this simulator.
  Which value reflects the real regulators is an open question for the professor.
- At 100 Hz the lag is a discrete pole at e^(−0.01/0.6) ≈ 0.983. Its corner frequency
  is 1/(2π·0.6) ≈ 0.27 Hz. It is the slowest, dominant dynamic in the plant.

### Pressure → force (exactly linear)

For column azimuth φ_s (S1 0°, S2 90°, S3 180°, S4 270°) and pouch level k:

```
M[k]       = pressure_gain × moment_arm × Σ_s P[s,k] · axis_s      (bending, about x and y)
F_axial[k] = extension_gain × Σ_s P[s,k]                          (stretch)
pressure_gain = 0.1274 N/psi, moment_arm = 0.028 m, extension_gain = 0.0919 N/psi
```

- Pouch k (P1..P5) acts **only** on level k.
- Opposing columns cancel in bending, so the 20 inputs reduce to 10 independent
  bending moments (x and y at each of 5 levels) plus 5 axial forces (the shared
  pressure at each level). The notebook's fixed 0.70–1.30 profile is just one choice
  of this allocation.
- Measured tip tendency per column: S1 → +x, S2 → +y, S3 → −x, S4 → −y.

### Mechanics

- 5 nested bodies `level0..level4`, one per pouch level, each 59.2 mm tall
  (rest length 0.296 m). Each level has, in order, `ext k` (axial slide along −z,
  range −5…+30 mm), `bx k` and `by k` (hinges about x and y, unbounded): 15 DOF.
- Linear springs/dampers, zero spring reference (relaxes to straight):
  bending 0.4168 N·m/rad and 0.3629 N·m·s/rad per hinge; axial 934.2 N/m and
  61.51 N·s/m per level.
- Masses: arm 0.35 kg (split across levels), tip marker frame 0.08 kg.
- The arm hangs: `level0` is attached to the mount at z = 0.576 m; the tip site
  rests at z = 0.268 m. Nonlinearity comes only from geometry and gravity, and is
  mild at ±15 mm.
- The model is a discretized approximation of piecewise-constant curvature (PCC):
  5 independently rotating rigid links, so an analytic PCC model will not match it
  exactly.
- `set_pre_inflation(p)` exists (stiffens joints with baseline pressure) but
  `make_sim()` leaves it at 0.

See [`figures/kinematic_diagram.png`](figures/kinematic_diagram.png)
(regenerate with `uv run python scripts/draw_kinematics.py`).

### P1 is at the mount

The notebook calls P1 the "bottom" pouch, which is true when the arm is drawn
upright (as on the slides). In the hanging simulation **P1 drives `level0`, next to
the mount, and P5 is nearest the tip.** Lower-numbered pouches move the tip more
because their bending is carried by the whole chain below: 8 psi on S1 P1 moved
the tip 3.4 mm, versus 1.4 mm for S1 P5 (5 s hold, other pouches at 2 psi).

### Sensor model

```
pouch_pressures = p_actual + curvature_coupling[k] · κ + extension_coupling[k] · ext + noise
curvature_coupling = [-3.111, -3.333, -1.903, -1.261, 0.228] psi/rad   (P1..P5)
extension_coupling = 0
noise σ = 0.002 psi
```

`segment_pressures` is the mean over a segment's five pouches.

### The MuJoCo model is generated, not a file

There is no `.xml` in the package. `soft_robotic_arm/model.py::build_arm_xml(cfg)`
returns the MJCF as a string built from the calibration. Pressure is applied
outside MuJoCo by `SoftArmSim._apply_pressure_wrench()` (`simulator.py`), so the
XML alone has no actuators. Loaded on its own (e.g.
`uv run python -m mujoco.viewer --mjcf soft_arm.xml`) it just hangs; only
`SoftArmSim` makes pressure act.

## Tools in this repo

- `uv run python scripts/view_arm.py` — runs `make_sim()` in MuJoCo's passive viewer
  with a Tk slider window: one slider per pouch (rows P1 at the mount → P5, columns
  S1–S4), a per-segment "all" slider, and reset. Sliders, not keys, because MuJoCo's
  viewer binds 0–5 (geom groups) and most letters (Q camera, W wireframe,
  E equality, R reflection, …). Viewer tips: double-click a body and Ctrl+drag to
  push it ("perturb").
- `uv run python scripts/view_arm.py --dump-xml soft_arm.xml` — write the MJCF.
- `uv run python scripts/draw_kinematics.py` — regenerate the kinematic diagram.
- `uv run python scripts/run_baseline.py` — reproduce the 4.10 mm PD baseline.

## Performance

About 1,100 control steps/s on one core (≈ 11× real time) on the development
laptop, so 1 M steps ≈ 15 min per core. Relevant for system-identification sweeps
and RL.
