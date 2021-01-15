.. _ln-build:

Building COSM
=================

.. toctree::
   :maxdepth: 2
   :caption: Contents:


CMake Variables
---------------

COSM requires the following CMake variables to be defined in order to
successfully build.

- ``COSM_BUILD_FOR`` - The target platform that COSM will be built for. Must be
  one of [ ``MSI``, ``ARGOS``, ``EV3`` ]. Setting this also sets
  ``COSM_HAL_TARGET`` and ``COSM_PROJECT_DEPS_PREFIX``.

- ``COSM_HAL_TARGET`` - Specify the Hardware Abstraction Layer (HAL)
  target. Must be one of: [ ``argos-footbot``, ``lego-ev3`` ].

- ``COSM_ARGOS_ROBOT_TYPE`` - The name of the type of robots within the swarm
  from the POV of ARGoS. Must match the type of robots in the XML input files
  fed to ARGoS!.

- ``COSM_ARGOS_ROBOT_NAME_PREFIX`` - The prefix that all robot names have within
  ARGoS. Must match the robot name prefix in the XML input files fed to ARGoS!.

- ``COSM_ARGOS_CONTROLLER_XML_ID`` - The unique name attached to the controller
  of the desired type in the XML input file to the the actual controller class
  in C++ code.

- ``COSM_PROJECT_DEPS_PREFIX`` - Prefix for where ARGoS and other dependencies
  in the parent project have been installed.
