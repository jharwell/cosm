.. Copyright 2022 John Harwell, All rights reserved.

.. _ln-cosm-support:

================================
Supported Platforms and Hardware
================================

.. _ln-cosm-support-pal:

Platform Abstraction Layer (PAL)
================================

COSM provides a common API for several *platforms* (a platform is an arbitrary
simulator, middleware layer, etc.) which client applications/libraries can build
against, leading to MUCH cleaner, more modular code; all the complexity is
handled in the PAL bindings for the platform. Currently supported platforms are:

- ARGoS (`<https://argos-sim.info>`_).

- ROS (`<https://ros.org>`_). For real robots *or* simulators which support ROS,
  such as Gazebo or WeBots.

.. _ln-cosm-support-hal:

Hardware Abstraction Layer (HAL)
================================

COSM provides a common API for several different agent/robot models within each
supported platform which client applications or libraries can build against to
increase code reuse and reduce the complexity of building for different
agents/robots. Currently supported hardware platforms are:

.. list-table::
   :header-rows: 1
   :widths: 10 80 10

   * - Parent Platform

     - Hardware/Agent Model

     - Support Maturity

   * - ARGoS

     - MarXbot (`<https://ieeexplore.ieee.org/document/5649153>`_); called the
       foot-bot in ARGoS.

     - Mature and nearly feature-complete.

   * - ARGoS

     - Drone
       (`<https://iridia.ulb.ac.be/IridiaTrSeries/link/IridiaTr2022-002.pdf>`_),
       a flying drone from `IRIDIA <https://code.ulb.ac.be/lab/IRIDIA>`_.

     - Beta. Some gaps in feature set such as setting target yaw in addition to
       position.

   * - ARGoS

     - Extended e-puck, based on the e-puck2
       (`<https://www.gctronic.com/e-puck2.php>`_).

     - Alpha. Does not build cleanly.

   * - ARGoS

     - Pi-puck (`<https://www.gctronic.com/doc/index.php?title=Pi-puck>`_), an
       e-puck with a Raspberry Pi mounted on it.

     - Alpha. Does not build cleanly.

   * - ROS

     - Extended Turtlebot3, based on the Turtlebot3
       (`<https://turtlebot.com>`_).

     - Beta. Many features available for other robots are missing.
