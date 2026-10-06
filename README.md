# soft-pneumatic-arm-control

RAS556 project: system identification and controller design for a four-segment,
twenty-pouch pneumatic soft robotic arm, simulated in MuJoCo via the
[`soft-robotic-arm`](https://pypi.org/project/soft-robotic-arm/) package
([source](https://github.com/Jeevan-HM/Soft-Robotic-Arm/tree/simulation/mujoco)).

Plan: identify a control-oriented model of the arm, start with a linear feedback
controller, then explore sliding mode control, and compare tracking accuracy and
robustness across reference trajectories.

## Setup

```bash
uv sync
```

## View the arm

```bash
uv run python scripts/view_arm.py                         # interactive MuJoCo viewer
uv run python scripts/view_arm.py --dump-xml soft_arm.xml  # write the generated MJCF
```

In the viewer, keys `1`–`4` raise segments S1–S4 by 0.5 psi, `Q W E R` lower
them, and `0` resets all segments to 2 psi.
