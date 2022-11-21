.. Copyright 2022 John Harwell, All rights reserved.

.. _ln-cosm-modules:

=====================
COSM Software Modules
=====================

High level summaries of COSM's main modules providing reusable (but not
necessarily generic functionality in MAS projects) are below; for full details
see the API.


- Artificial potential fields for navigation and flocking: :cosm:`cosm::apf2D`.

- Platform Abstraction Layer (PAL): :cosm:`cosm::pal`.

- Hardware Abstraction Layer (HAL): :cosm:`cosm::hal`.

- Task allocation. Various methods from the swarm robotics literature, most of
  which do not require communication.

- Flocking. Also metrics and collectors. :cosm:`cosm::flocking` and
  :cosm:`cosm::spatial::strategy::flocking`.

- Common agent subsystems: :cosm:`cosm::subsystem`

  - Sensing and Actuation (SAA)

  - Perception

- Foraging: block distribution, block motion, injection of perfect information
  about blocks and caches. Metrics and collectors. :cosm:`cosm::foraging`.

- General kinematics definitions for interfacing with ROS and other platforms:
  :cosm:`cosm::kin`.

- Convergence measurements of different types; metrics and
  collectors. :cosm:`cosm::convergence`.

- Injection of environmental and population dynamics; metrics and
  collectors. :cosm:`cosm::tv`.

- Simple management of arena state via grid: :cosm:`cosm::arena`.
