# RAS556 – Precision Control of a Soft Robotic Manipulator

Project proposal submitted by Nikoletta Biri and Muhammad Ibraheem (September 2026).
Text transcribed from the submitted Word file,
[`RAS556_project_proposal.doc`](RAS556_project_proposal.doc), with DOI links added
to the references.

**Abstract** — Soft robotic manipulators are flexible robotic systems that can adapt
to their environment and safely interact with objects. However, because of their
soft structure and pneumatic actuation, controlling their motion accurately can be
challenging. In this project, we plan to explore the control of a pneumatically
actuated soft robotic manipulator using the provided MuJoCo simulation model. Our
main goal is to understand how different pressure inputs affect the motion of the
manipulator and how feedback control can be used to follow a desired position or
trajectory. We plan to begin with a linear feedback controller and explore sliding
mode control as a possible method for improving tracking performance and
robustness. The controllers will be tested with different reference trajectories in
simulation, and their tracking performance will be compared.

**Keywords** — Soft Robotics, Pneumatic Manipulator, Trajectory Tracking, Feedback
Control, Sliding Mode Control, MuJoCo

## I. Introduction

We plan to work on the precision control of a soft robotic manipulator. We chose
this topic because, as robotics students, we would like to gain more experience in
the field of soft robotics and learn more about the challenges involved in
controlling flexible robotic systems. Soft robotic manipulators are flexible and
compliant, which can make them useful for applications involving delicate objects,
human interaction, or constrained environments. However, their flexibility also
makes accurate control more challenging than for traditional rigid robotic arms.
The goal of our project is to investigate how feedback control can be used to make
the provided pneumatic soft manipulator accurately follow desired positions and
trajectories. We want to study how pressure inputs can be controlled to reduce
tracking error while considering factors such as actuator dynamics, modeling
inaccuracies, and sensor noise.

Our project team consists of Nikoletta Biri and Muhammad Ibraheem. We plan to work
collaboratively on understanding the soft robotic manipulator model and developing
the control strategies. The work will be divided between controller implementation
and simulation, literature review and analysis of the results, with both team
members contributing to controller design, testing, documentation, and the final
presentation.

Precise control of soft robotic manipulators is challenging because their flexible
structure can make their behavior more difficult to model and predict than
traditional rigid robots. Previous research has explored different control
approaches for improving trajectory tracking in soft robots. Pilch, Menrad, Beger,
and Sawodny developed a dynamic model and control approach for a pneumatically
actuated soft continuum manipulator, using model-based feedforward control together
with PID feedback control to improve position tracking [1]. Fallahi, Zareinejad,
Rezaei, Ghafarirad, and Talebi developed an adaptive sliding mode control approach
for position tracking of a pneumatic soft manipulator, focusing on maintaining
tracking performance in the presence of modeling uncertainties and disturbances
[2]. Sliding mode control has also been investigated using data-driven modeling
approaches. Papageorgiou, Sigurðardóttir, Falotico, and Tolu used a sliding mode
controller for end-effector position control of a soft robot and demonstrated the
approach experimentally [3]. Zhi Qiao, Weijia Tao, and Wenlong Zhang also used a
nonlinear disturbance observer and sliding mode controller to track joint-space
trajectories on top of a linear parameter-varying model [4].

For our project, we plan to begin with a linear feedback controller to better
understand the behavior and control of the provided soft robotic manipulator. We
then plan to explore sliding mode control as a possible approach for improving
trajectory tracking and robustness. We will use the provided MuJoCo simulation to
test the controllers with different reference trajectories and compare their
performance based on tracking accuracy and their ability to handle disturbances and
uncertainties.

## References

1. S. Pilch, E. Menrad, A. Beger, and O. Sawodny, "Dynamic modeling and control of an
   inextensible pneumatically actuated soft continuum manipulator," *Mechatronics*,
   vol. 115, Art. no. 103462, 2026.
   <https://doi.org/10.1016/j.mechatronics.2026.103462>
2. M. Fallahi, M. Zareinejad, S. M. Rezaei, H. Ghafarirad, and H. A. Talebi, "Adaptive
   sliding mode control for quasi-static position tracking of a pneumatic soft
   manipulator," *European Journal of Control*, vol. 91, Part 2, Art. no. 101548, 2026.
   <https://doi.org/10.1016/j.ejcon.2026.101548>
3. D. Papageorgiou, G. Þ. Sigurðardóttir, E. Falotico, and S. Tolu, "Sliding-mode
   control of a soft robot based on data-driven sparse identification," *Control
   Engineering Practice*, vol. 144, Art. no. 105836, 2024.
   <https://doi.org/10.1016/j.conengprac.2023.105836>
4. Z. Qiao, W. Tao, and W. Zhang, "Nonlinear disturbance observer with sliding mode
   control for a fabric soft robotic arm," *IFAC-PapersOnLine*, vol. 58, no. 28,
   pp. 516–521, 2024. <https://doi.org/10.1016/j.ifacol.2025.01.098>
