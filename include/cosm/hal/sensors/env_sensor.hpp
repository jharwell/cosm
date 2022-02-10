/**
 * \file env_sensor.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/hal/hal.hpp"
#include "cosm/cosm.hpp"
#include "cosm/hal/sensors/config/env_sensor_config.hpp"

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
#include "cosm/hal/argos/sensors/ground_sensor.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/sensors/light_sensor.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */


/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using env_sensor_impl = chargos::sensors::ground_sensor;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using env_sensor_impl = chros::sensors::light_sensor;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/**
 * \class env_sensor
 * \ingroup hal sensors
 *
 * \brief Environment sensor wrapper to provide a uniform interface to sensing
 * (roughly) "what is the environment/spatial features of the environment/etc
 * near me" regardless of robot and platform. Provides some
 * additional higher level functionality beyond raw sensor readings too.
 */
class env_sensor final : public rer::client<env_sensor>,
                         public env_sensor_impl {
 public:
  static inline const std::string kNestTarget = "nest";

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
  template <typename TSensor>
    env_sensor(TSensor * const sensor,
               const chsensors::config::env_sensor_config* const config)
       : ER_CLIENT_INIT("cosm.hal.sensors.env"),
         env_sensor_impl(sensor, config) {}
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
  explicit env_sensor(const cros::topic& robot_ns,
                      const config::env_sensor_config* const config)
      : ER_CLIENT_INIT("cosm.hal.sensors.env"),
        env_sensor_impl(robot_ns) {}
#endif

  /* move only constructible/assignable to work with the saa subsystem */
  env_sensor(env_sensor&&) = default;
  env_sensor& operator=(env_sensor&&) = default;
  env_sensor(env_sensor&) = delete;
  env_sensor& operator=(env_sensor&) = delete;
};

NS_END(sensors, hal, cosm);

