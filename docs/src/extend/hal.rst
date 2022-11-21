.. SPDX-License-Identifier:  MIT

.. _ln-cosm-extend-hal:

==================================
Adding A New Agent Type To The HAL
==================================

Below is a ROUGH outline of how to add a new agent type to an already existing
platform. In all likelihood, the steps below will be insufficient, so please
update these docs as you go.

#. Define your agent in ``cmake/hal.cmake``:

   - Define members of ``COSM_HAL_TARGET_SENSOR_SUITE`` and
     ``COSM_HAL_TARGET_ACTUATOR_SUITE`` as appropriate.

   - Define whether the agent operates in 2D or 3D with
     ``COSM_HAL_TARGET_OPERATES_IN_3D`` or
     ``COSM_HAL_TARGET_OPERATES_IN_Q3D``. There is not "operates in 2D" mode,
     because all agents sense in 3D if the underlying PAL platform supports
     in. If it doesn't, then the Z component is just zeroed everywhere.

#. Add a new ``#define`` in ``cosm/hal/hal.hpp``, following the convention of::

     #define COSM_HAL_TARGET_<PLATFORM>_<AGENT TYPE>

#. For each sensor that your agent is equipped with, add an appropriate sensor
   in the HAL if one does not already exist under
   ``cosm/hal/<platform>/sensors``. If one already exist, add your agent type to
   the set of agent types which define that sensor/actuator.

   .. IMPORTANT:: Each agent type MUST define the following sensors even if they
      are actually not present on the agent:

      - :cosm:`cosm::hal::sensors::env_sensor`
      - :cosm:`cosm::hal::sensors::light_sensor`
      - :cosm:`cosm::hal::sensors::proximity_sensor`
      - :cosm:`cosm::hal::sensors::odometry_sensor`

      If the agent doesn't have the sensor, then
      :cosm:`cosm::hal::sensors::stub_sensor` can be used.

   There is a lot of stuff under ``spatial/`` which depends one or more of these
   sensors, and requiring all agents "support" this set of sensors seemed like
   the lesser of two evils. Other options considered, but rejected for the
   moment:

     - Put ``#if defined`` guards around sensor usage causing the errors. This
       violates the general ethos that all such guards and other trickery should
       be confined to the HAL.

   This decision may be revisited in the future.

#. For each actuator that your agent is equipped with, add an appropriate actuator
   in the HAL if one does not already exist under
   ``cosm/hal/<platform>/sensors``. If one already exist, add your agent type to
   the set of agent types which define that sensor/actuator.

   .. IMPORTANT:: Each agent type MUST define the following actuators:

      - :cosm:`cosm::hal::actuators::locomation_actuator`

#. Under ``cosm/hal/<platform>/subsystem/robot_available_sensors.hpp`` add an
   entry for your agent to define the sensors that it has. If you don't define
   the sensors here, you will get all kinds of cryptic errors when building!

#. Under ``cosm/hal/<platform>/subsystem/robot_available_actuators.hpp`` add an
   entry for your agent to define the actuators that it has. If you don't define
   the actuators here, you will get all kinds of cryptic errors when building!

#. Try to build, fixing the inevitable compiler errors. If no errors--hooray for
   you! If you are less lucky, fix the errors, keeping in mind the following:

   - Many seemingly intractable problems of the nature of "these things just
     don't fit together" can be solved via a layer of indirection somewhere.

   If there are files which just won't compile, consider modifying the contents
   of the COSM components to not build them. This is LAST RESORT: COSM has been
   built with enough different agent and platform types that things are
   generally organized well enough that it should be possible to accommodate any
   additional agent type.

#. If you have changed any of the XML parsers, update the :ref:`controller
   <ln-cosm-usage-xml-controllers>` and/or :ref:`swarm manager
   <ln-cosm-usage-xml-swarm-manager>` documentation.

#. Update the master table of the stuff that HAL/PAL support in the README.
