# Assignment context

## Course and team

- Course: RAS556, Fall 2026. The arm, its simulator and the coursework notebook
  come from Wenlong Zhang's lab at Arizona State University (the simulator author,
  Jeevan Hebbal Manjunath, is a PhD student there).
- Team: Nikoletta Biri ([@Niki-89-AI](https://github.com/Niki-89-AI)) and
  Muhammad Ibraheem ([@ibraheem-111](https://github.com/ibraheem-111)).
- Our proposal: [`proposal/proposal.md`](proposal/proposal.md) (original Word file
  alongside it). Scope in one line: understand how pressure inputs move the arm,
  build a linear feedback controller, then explore sliding mode control (SMC), and
  compare controllers on several reference trajectories for accuracy and for
  robustness to disturbances and model uncertainty.

## The professor's coursework notebook

Our copy: [`notebooks/soft_robotic_arm.ipynb`](../notebooks/soft_robotic_arm.ipynb)
([open in Colab](https://colab.research.google.com/github/ibraheem-111/soft-pneumatic-arm-control/blob/main/notebooks/soft_robotic_arm.ipynb)).
It is the professor's notebook as copied into Colab (outputs stripped on commit).
Upstream original:
[`mujoco/coursework/soft_robotic_arm_new.ipynb`](https://github.com/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/coursework/soft_robotic_arm_new.ipynb)
([open in Colab](https://colab.research.google.com/github/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/coursework/soft_robotic_arm_new.ipynb)).

What the notebook teaches:

1. Command all twenty pouches independently (`(4, 5)` command, psi).
2. Read the five pouch pressures in every segment.
3. Observe the tip position and velocity.
4. Use a PD controller to track a smooth random tip trajectory.
5. Render the controller's motion as a video (optional).

### Student tasks (from the upstream notebook)

Our Colab copy is a slightly newer revision that dropped this cell; the upstream
file still has it:

- Tune `kp` and `kd` and explain their effect on tracking.
- Change `trajectory_seed`, `span_m`, or the frequencies and compare the RMSE.
- Change `pouch_profile` to investigate how P1–P5 affect the motion.
- Keep every command between 0 and 9 psi.
- Explain how the `(4, 5)` command maps to the twenty pouches.

### Controller contract

Any controller we write must keep the notebook's interface so results are
comparable:

```python
class MyController:
    def reference(self, t) -> tuple[np.ndarray, np.ndarray]: ...   # desired xy position [m], velocity [m/s]
    def compute(self, t: float, obs: dict) -> np.ndarray: ...       # finite (4, 5) absolute pouch pressures, psi, within 0..9
```

The loop is measure → `compute` → `sim.step(command)`; the controller must never
call `sim.step()` itself.

### Baseline to beat

`RandomTrajectoryPDController` in the notebook:

- Cartesian (tip x–y) PD: Δp_xy = Kp (r_d − r) + Kd (v_d − v), with Kp = 500 psi/m
  and Kd = 20 psi·s/m.
- 4.5 psi bias on every pouch; +Δp_x on S1 and −Δp_x on S3, +Δp_y on S2 and −Δp_y on S4.
- Static pouch allocation: each segment's correction is scaled by
  `pouch_profile = [0.70, 0.85, 1.00, 1.15, 1.30]` across P1..P5, then clipped to 0..9 psi.
- Reference: sum of sinusoids at 0.05, 0.09 and 0.14 Hz with random amplitudes and
  phases (trajectory seed 7, span 15 mm), centred on the measured start position.
- Benchmark run: `make_sim(control_hz=100, seed=2)`, 20 s, RMSE over the whole run
  (includes the initial transient).
- **Result: 4.10 mm planar RMSE.** Reproduced locally with
  `uv run python scripts/run_baseline.py` (soft-robotic-arm 0.4.2).

Use the same sim seed, trajectory seed and duration when comparing new controllers.
