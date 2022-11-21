.. Copyright 2022 John Harwell, All rights reserved.

.. _ln-cosm-setup:

===============
Setting Up COSM
===============

User Setup
==========

#. Install/Setup Dependencies

   - :ref:`RCPPSW user setup <ln-rcppsw-setup-user>`

#. Install COSM package(s).

Developer Setup
===============

#. COSM uses `LIBRA <https://jharwell.github.io/libra>`_ so go to
   :ref:`ln-libra-req` and install any needed packages.

#. Setup COSM's dependencies (packages is fine, unless you'll be modifying the
   dependencies too):

   - :ref:`RCPPSW user setup <ln-rcppsw-setup-user>`


#. Clone COSM and init LIBRA::

     git clone git@github.com:jharwell/cosm.git
     cd cosm
     git submodule update --init --remote --recursive

#. Build COSM. From the root of the repo::

     mkdir build && cd build
     cmake <ARGS> ..

   ``<ARGS>`` is a list of cmake arguments.

   .. IMPORTANT:: COSM and LIBRA output **VERY** thorough summaries of their
                  build configuration, so check them to make sure you are
                  building what you think you are.


   You can pass any option as part of ``<ARGS>`` that LIBRA supports (see
   :ref:`ln-libra-capabilities`). In addition, you must specify what
   platform+agent type you will built COSM for via ``COSM_BUILD_FOR``. Options
   are as follows (see :ref:`ln-cosm-support` for details):

   .. list-table::
      :header-rows: 1
      :widths: 10 10

      * - Platform+Hardware/Agent Model

        - ``COSM_BUILD_FOR`` value

      * - ARGOS Foot-bot

        - ARGOS_FOOTBOT

      * - ARGoS Drone

        - ARGOS_DRONE

      * - ARGoS E-puck

        - ARGOS_EEPUCK3D

      * - ARGoS Pi-puck

        - ARGO_PIPUCK

      * - Extended TURTLEBOT3 with ROS

        - ROS_ETURTLEBOT3

   Some example build commands and their meaning:

   .. list-table::
      :header-rows: 1
      :widths: 10 90

      * - Command

        - Meaning

      * -

          ::

             cmake \
             -DCOSM_BUILD_FOR=ARGOS_FOOTBOT

        - Build for the ARGoS foot-bot, development build with default event
          reporting/logging (inherited from RCPPSW).

      * -

          ::

            cmake \
            -DCOSM_BUILD_FOR=ARGOS_FOOTBOT \
            -DCMAKE_BUILD_TYPE=OPT \
            -DLIBRA_ER=NONE

        - Build for the ARGoS foot-bot, optimized build with no event reporting.

      * -

          ::

             cmake \
             -DCOSM_BUILD_FOR=ARGOS_DRONE \
             -DCMAKE_INSTALL_PREFIX=$HOME/.local

        - Build for the ARGoS drone, development build with default event
          reporting, installing to a different location.
