# References and external resources

DOIs and arXiv IDs were checked against Crossref and the arXiv API (October 2026).
PDFs are not committed because most are copyrighted; use the links (ASU library
access works for the paywalled ones).

## Course material and code

| What | Link |
|---|---|
| This repo | <https://github.com/ibraheem-111/soft-pneumatic-arm-control> |
| Our copy of the coursework notebook | [`notebooks/soft_robotic_arm.ipynb`](../notebooks/soft_robotic_arm.ipynb) · [Open in Colab](https://colab.research.google.com/github/ibraheem-111/soft-pneumatic-arm-control/blob/main/notebooks/soft_robotic_arm.ipynb) |
| Upstream coursework notebook | [GitHub](https://github.com/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/coursework/soft_robotic_arm_new.ipynb) · [Open in Colab](https://colab.research.google.com/github/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/coursework/soft_robotic_arm_new.ipynb) |
| Simulator source (branch `simulation`) | <https://github.com/Jeevan-HM/Soft-Robotic-Arm/tree/simulation/mujoco> |
| Simulator on PyPI | <https://pypi.org/project/soft-robotic-arm/> |
| Upstream model docs | [arm_model_guide.md](https://github.com/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/docs/arm_model_guide.md) · [arm_parameters.md](https://github.com/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/docs/arm_parameters.md) · [hardware_integration.md](https://github.com/Jeevan-HM/Soft-Robotic-Arm/blob/simulation/mujoco/docs/hardware_integration.md) |
| Lab hardware repo (branch `main`) | <https://github.com/Jeevan-HM/Soft-Robotic-Arm> |
| Our proposal | [`proposal/proposal.md`](proposal/proposal.md) |

## Papers cited in our proposal

1. S. Pilch, E. Menrad, A. Beger, O. Sawodny, "Dynamic modeling and control of an
   inextensible pneumatically actuated soft continuum manipulator," *Mechatronics*
   115, 103462, 2026. <https://doi.org/10.1016/j.mechatronics.2026.103462> —
   model-based feedforward plus PID feedback.
2. M. Fallahi et al., "Adaptive sliding mode control for quasi-static position
   tracking of a pneumatic soft manipulator," *European Journal of Control* 91,
   101548, 2026. <https://doi.org/10.1016/j.ejcon.2026.101548> — adaptive SMC under
   model uncertainty.
3. D. Papageorgiou, G. Þ. Sigurðardóttir, E. Falotico, S. Tolu, "Sliding-mode control
   of a soft robot based on data-driven sparse identification," *Control Engineering
   Practice* 144, 105836, 2024. <https://doi.org/10.1016/j.conengprac.2023.105836> —
   SINDy model + SMC, the closest template for "identify, then SMC".
4. Z. Qiao, W. Tao, W. Zhang, "Nonlinear disturbance observer with sliding mode
   control for a fabric soft robotic arm," *IFAC-PapersOnLine* 58(28), 516–521, 2024.
   <https://doi.org/10.1016/j.ifacol.2025.01.098> — **the robot's own lab** (ASU).
   First-order LPV model + fast nonlinear disturbance observer + SMC on a two-segment
   fabric arm. Assumes the inner pressure loop is fast enough to ignore — not true in
   our simulator (τ = 0.6 s), so the lag must be modelled. Open access
   (CC BY-NC-ND).

## Further papers from our notes

- D. Bruder, X. Fu, R. B. Gillespie, C. D. Remy, R. Vasudevan, "Data-Driven Control
  of Soft Robots Using Koopman Operator Theory," *IEEE Transactions on Robotics*
  37(3), 948–961, 2021. <https://doi.org/10.1109/TRO.2020.3038693> — EDMD lifting
  with delays, linear and nonlinear Koopman MPC on a pneumatic soft arm; Koopman
  controllers > 3× more accurate than a linear state-space MPC benchmark.
- D. Bruder, B. Gillespie, C. D. Remy, R. Vasudevan, "Modeling and Control of Soft
  Robots Using the Koopman Operator and Model Predictive Control," 2019.
  [arXiv:1902.02827](https://arxiv.org/abs/1902.02827) — the earlier conference
  version.
- S. L. Brunton, J. L. Proctor, J. N. Kutz, "Discovering governing equations from
  data by sparse identification of nonlinear dynamical systems," *PNAS* 113,
  2016. <https://doi.org/10.1073/pnas.1517384113> — SINDy.
- S. L. Brunton, J. L. Proctor, J. N. Kutz, "Sparse Identification of Nonlinear
  Dynamics with Control (SINDYc)," 2016.
  [arXiv:1605.06682](https://arxiv.org/abs/1605.06682).
- J. Hebbal Manjunath, J. Wang, S. Li, W. Zhang, "Towards Effective Physical
  Reservoir Computing with a Pneumatic Soft Robot," 2026.
  [arXiv:2609.02157](https://arxiv.org/abs/2609.02157) — by the simulator's author,
  on this lab's arm.
- R. J. Webster III, B. A. Jones, "Design and Kinematic Modeling of Constant
  Curvature Continuum Robots: A Review," *IJRR* 29, 2010.
  <https://doi.org/10.1177/0278364910368147> — PCC kinematics.
- Zheng, Burgner-Kahrs, "Estimating dynamic soft continuum robot states from
  boundaries," *IJRR*, 2026. <https://doi.org/10.1177/02783649261483674> — listed in
  our notes as "State Estimation of Continuum Robots".
- T. Nguyen, W. Zhang, "Design and Computational Modeling of Fabric Soft Pneumatic
  Actuators for Wearable Assistive Devices," *Scientific Reports*, 2020.
  <https://doi.org/10.1038/s41598-020-65003-2> — the pouch/actuator design.
- C. Della Santina, C. Duriez, D. Rus, "Model-Based Control of Soft Robots: A Survey
  of the State of the Art and Open Challenges," *IEEE Control Systems Magazine*
  43, 30–65, 2023. <https://doi.org/10.1109/MCS.2023.3253419>
- M. Falkenhahn, A. Hildebrandt, R. Neumann, O. Sawodny, "Model-based feedforward
  position control of constant curvature continuum robots using feedback
  linearization," *ICRA* 2015. <https://doi.org/10.1109/ICRA.2015.7139264>

## System-identification theory

Recommended reading for the identification experiments
(see [`modeling-plan.md`](modeling-plan.md)):

- L. Ljung, *System Identification: Theory for the User*, 2nd ed., Prentice Hall,
  1999. The standard text: Ch. 4 model structures (ARX, ARMAX, OE, state-space),
  Ch. 7 prediction-error methods, Ch. 13 experiment design (PRBS, multisines,
  persistent excitation), Ch. 16 model validation.
- T. Söderström, P. Stoica, *System Identification*, Prentice Hall, 1989. Same
  ground, more mathematical; the authors distribute a free PDF.
- R. Pintelon, J. Schoukens, *System Identification: A Frequency Domain Approach*,
  2nd ed., Wiley–IEEE Press. Multisine design, frequency-response estimation, detecting
  nonlinear distortion.
- S. L. Brunton, J. N. Kutz, *Data-Driven Science and Engineering*, Cambridge Univ.
  Press. Free online at <https://databookuw.com>. Chapters on DMD, SINDy and Koopman.
- MIT OpenCourseWare 6.435 *System Identification* — lecture notes.

These were recommended from general knowledge and have not been checked against
any RAS556 reading list.
