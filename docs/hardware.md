# The physical robot

Transcribed from the lab's presentation slides ("Robot Design", "Hardware &
Electronics Design", "Dynamics I — Structural Model", "Dynamics II — Actuation &
Simulation"), which Ibraheem received as screenshots. The slides themselves are
not in this repo. Where a slide disagrees with the simulator code, the code wins
for simulation work; the disagreement is noted below.

## Robot design

- Four TPU-pouch columns around the centreline at 0°/90°/180°/270° (E/N/W/S). Each
  column is split into 5 stacked pouches: **20 individually addressable channels**,
  rated to 10 psi.
- Each pouch is a heat-sealed TPU bladder inside a constraining fabric sleeve, so
  inflation elongates the column instead of ballooning.
- The arm hangs from a plywood mounting plate on a 4-post aluminium-extrusion frame.
  Tubing for all 20 channels runs down the centreline to the regulators below.
- A tip-mounted marker frame carries OptiTrack retroreflective markers for real-time
  6-DOF pose. It was used both to fit the model's parameters and as ground truth for
  controller validation.
- Actuator design paper:
  [Nguyen & Zhang, Sci. Rep. 2020](https://www.nature.com/articles/s41598-020-65003-2).
  Robot paper: [Qiao, Tao & Zhang, IFAC-PapersOnLine 2024](https://doi.org/10.1016/j.ifacol.2025.01.098).

## Electronics

- **Digital command:** a Raspberry Pi computes the 20-channel pressure command at
  100 Hz and sends it over I2C to MCP4728 DACs (4 channels each), one analog command
  voltage per pouch.
- **Pneumatic actuation:** each voltage drives an SMC ITV1050 electro-pneumatic
  regulator, which regulates the shared compressed-air supply to the commanded
  pressure for its pouch. One regulator per pouch, so per-pouch control is real on
  the hardware, not a simulation-only feature.
- **Feedback:** ADS1115 ADCs read the delivered pressure back to the Pi for
  verification, logging and sensor-model calibration.

## Model description on the slides, checked against the code

| Slide says | `soft-robotic-arm` 0.4.2 |
|---|---|
| Discretized PCC chain: 5 rigid bodies, each with `ext_k`, `bx_k`, `by_k` (15 DOF) | Matches |
| Passive stiffness and damping on every joint; relaxes to straight | Matches |
| Pressure → generalized force via `qfrc_applied`; bending linear in pressure, axial ∝ total level pressure | Matches |
| 1 kHz `implicitfast` physics, 100 Hz control, 10 substeps | Matches |
| **First-order lag τ ≈ 120 ms** | **τ = 0.6 s** (changed 2026-09-30; see [`simulator.md`](simulator.md)) |
| Pre-inflation stiffens joints | Implemented (`set_pre_inflation`) but `make_sim()` leaves it at 0 |
| Sensor model adds curvature and extension coupling plus noise | Extension coupling is 0; curvature coupling and noise only |
| Pouches rated to 10 psi | Coursework command limit is 9 psi |

The slides also flag a modelling caveat worth repeating in our report: true PCC
couples bending angle and arc length nonlinearly, while 5 independently rotating
rigid links only approximate that curve. An analytic PCC model will therefore not
match the simulator exactly.
