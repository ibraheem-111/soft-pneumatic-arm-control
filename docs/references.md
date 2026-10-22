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
   SINDy model + SMC, the closest template for "identify, then SMC". Note the robot is
   cable/McKibben-driven with servo-count inputs, and the model has no actuator lag.
4. Z. Qiao, W. Tao, W. Zhang, "Nonlinear disturbance observer with sliding mode
   control for a fabric soft robotic arm," *IFAC-PapersOnLine* 58(28), 516–521, 2024.
   <https://doi.org/10.1016/j.ifacol.2025.01.098> — **the robot's own lab** (ASU).
   First-order LPV model under PCC + fast nonlinear disturbance observer + SMC on a
   **two-segment** fabric arm (front/back columns held at 2 psi, left/right
   modulated), so an earlier robot than ours. Stiffness and damping were identified
   per set point by least squares, then fitted with polynomials. Read in full
   (2026-10-12, PDF pages viewed): licence CC BY-NC-ND. State q = [θ₁, L₁, θ₂, L₂]
   (PCC bending angle and arc length per segment, from wire encoders), not tip
   position. Inertia, Coriolis and gravity are lumped into a disturbance d, giving the
   first-order (massless) model q̇ = A(ρ)q + B(ρ)u + d with A = −diag(kᵢ/cᵢ), kᵢ, cᵢ
   quadratic in a pressure-derived scheduling variable ρ (eq. 2, 17). SMC surface
   σ = e (relative degree 1), adaptive switching gain; NDOB with gain
   l₁ + 2l₂|q| (eqs. 6–8) supplies d̂ in u = −c(−q̇_d + aq + λσ + η sat(σ) + d̂).
   Input u is a torque/force; Fig. 2 has an inner loop (torque/force controller →
   pressure controller) that is **assumed fast enough to neglect**, so pressure
   dynamics are not modelled. Experiments at 40 Hz, θ_d = 0.35 sin²(2πt/30), seven
   trials. Our simulator's τ = 0.6 s lag (plus the 0.5 s delay on the calibrated
   plant) is not fast relative to a 0.1 Hz task, so that assumption needs checking
   here.

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

## Pressure dynamics as a first-order linear ODE

- C. C. Johnson, D. G. Cheney, D. L. Cordon, M. D. Killpack, "PneuDrive: An Embedded
  Pressure Control System and Modeling Toolkit for Large-Scale Soft Robots," arXiv:2504.00222,
  2025. <https://arxiv.org/abs/2504.00222>. §II-B1 gives the linear pressure model
  ṗ = −αp + βp_cmd (α, β tuned), attributing it to Best et al. 2016, who chose it from step
  responses of a PID pressure controller; it also gives a nonlinear first-order model from
  mass flow and chamber volume. Ours is the same form: α = 1/τ, β = g/τ, plus the dead time.
  Checked 2026-10-21 by reading the arXiv HTML.
- C. M. Best, M. T. Gillespie, P. Hyatt, L. Rupert, V. Sherrod, M. D. Killpack, "A new soft
  robot control method: Using model predictive control for a pneumatically actuated
  humanoid," *IEEE Robotics & Automation Magazine* 23(3), 75–84, 2016.
  <https://scholarsarchive.byu.edu/facpub/3208>. The original use of that model (per
  PneuDrive); full text not read, bibliographic details confirmed via DBLP / BYU.

## Sources cited in `docs/reports/model-discussion-guide.html`

Existence of each checked on 2026-10-21 via DOI/Crossref/arXiv/DBLP (section locations not all verified).

- Z. Qiao, W. Tao, W. Zhang, "Nonlinear disturbance observer with sliding mode control for a fabric soft robotic arm," IFAC-PapersOnLine 58(28):516–521, 2024. doi:10.1016/j.ifacol.2025.01.098
- R. J. Webster III, B. A. Jones, "Design and kinematic modeling of constant curvature continuum robots: a review," IJRR 29(13):1661–1683, 2010. doi:10.1177/0278364910368147
- C. Della Santina, C. Duriez, D. Rus, "Model-based control of soft robots: a survey," IEEE CSM 43(3):30–65, 2023. doi:10.1109/MCS.2023.3253419; arXiv:2110.01358
- C. C. Johnson, D. G. Cheney, D. L. Cordon, M. D. Killpack, "PneuDrive: an embedded pressure control system and modeling toolkit for large-scale soft robots," arXiv:2504.00222, 2025.
- C. M. Best, M. T. Gillespie, P. Hyatt, L. Rupert, V. Sherrod, M. D. Killpack, "A new soft robot control method: using model predictive control for a pneumatically actuated humanoid," IEEE RAM 23(3):75–84, 2016. doi:10.1109/MRA.2016.2580591
- L. Ljung, System Identification: Theory for the User, 2nd ed., Prentice Hall, 1999.
- J. Schoukens, L. Ljung, "Nonlinear system identification: a user-oriented road map," IEEE CSM 39(6):28–99, 2019. doi:10.1109/MCS.2019.2938121; arXiv:1902.00683
- R. Pintelon, J. Schoukens, System Identification: A Frequency Domain Approach, 2nd ed., Wiley, 2012. doi:10.1002/9781118287422
- J. Schoukens, R. Pintelon, T. Dobrowiecki, Y. Rolain, "Identification of linear systems with nonlinear distortions," Automatica 41(3):491–504, 2005. doi:10.1016/j.automatica.2004.10.004
- K. Vanhoenacker, T. Dobrowiecki, J. Schoukens, "Design of multisine excitations to characterize the nonlinear distortions during FRF-measurements," IEEE TIM 50(5):1097–1102, 2001. doi:10.1109/19.963166
- T. K. Caughey, "Classical normal modes in damped linear dynamic systems," J. Appl. Mech. 27(2):269–271, 1960. doi:10.1115/1.3643949
- W. J. Rugh, J. S. Shamma, "Research on gain scheduling," Automatica 36(10):1401–1425, 2000. doi:10.1016/S0005-1098(00)00058-3
- V. I. Utkin, "Variable structure systems with sliding modes," IEEE TAC 22(2):212–222, 1977. doi:10.1109/TAC.1977.1101446
- J.-J. E. Slotine, W. Li, Applied Nonlinear Control, Prentice Hall, 1991 (ch. 7, sliding control).
- C. Edwards, S. Spurgeon, Sliding Mode Control: Theory and Applications, Taylor & Francis, 1998. doi:10.1201/9781498701822
- O. J. M. Smith, "Closer control of loops with dead time," Chem. Eng. Prog. 53(5):217–219, 1957.
- J. E. Normey-Rico, E. F. Camacho, Control of Dead-time Processes, Springer, 2007, ch. 5. doi:10.1007/978-1-84628-829-6
- G. M. Clayton, S. Tien, K. K. Leang, Q. Zou, S. Devasia, "A review of feedforward control approaches in nanopositioning for high-speed SPM," J. Dyn. Sys. Meas. Control 131(6), 2009. doi:10.1115/1.4000158
- W.-H. Chen, D. J. Ballance, P. J. Gawthrop, J. O'Reilly, "A nonlinear disturbance observer for robotic manipulators," IEEE TIE 47(4):932–938, 2000. doi:10.1109/41.857974
- K.-S. Kim, K.-H. Rew, S. Kim, IEEE TAC 55(8):1905–1911, 2010 (higher-order disturbance observer, cited by the paper). doi:10.1109/TAC.2010.2049522
