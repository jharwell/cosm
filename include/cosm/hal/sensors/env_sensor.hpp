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

#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/cosm.hpp"
#include "cosm/hal/sensors/config/env_sensor_config.hpp"

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
#include "cosm/hal/argos/sensors/env_sensor.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/sensors/env_sensor.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */


/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using impl_handle = chargos::sensors::env_sensor;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using impl_handle = chros::sensors::env_sensor;
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
                         public rpdecorator::decorator<impl_handle> {
 public:
  static inline const std::string kNestTarget = "nest";
  static inline const std::string kBlockTarget = "block";

  using rpdecorator::decorator<impl_handle>::decoratee;

  env_sensor(impl_handle&& impl,
             const config::env_sensor_config* const config)
       : ER_CLIENT_INIT("cosm.hal.sensors.env"),
         decorator(std::move(impl)),
         m_config(*config) {}

  /* move only constructible/assignable to work with the saa subsystem */
  env_sensor(env_sensor&&) = default;
  env_sensor& operator=(env_sensor&&) = default;
  env_sensor(env_sensor&) = delete;
  env_sensor& operator=(env_sensor&) = delete;

  RCPPSW_DECORATE_DECLDEF(reset);
  RCPPSW_DECORATE_DECLDEF(disable);

  /**
   * \brief Detect if a certain condition is met by examining sensor
   * readings.
   *
   * \param name The name of the configured detection to check.
   *
   * \return \c TRUE iff the condition was detected.
   */
  bool detect(const std::string& name) {
    const auto &detection = m_config.detect_map.find(name);

    ER_ASSERT(m_config.detect_map.end() != detection,
              "Detection %s not found in configured map: cannot detect",
              name.c_str());

    if (!detection->second.enabled) {
      ER_WARN("Environmental feature detection of %s disabled",
              name.c_str());
      return false;
    }
    return decoratee().detect(name, &detection->second);
  }

  void disable(const std::string& name) {
    auto detection = m_config.detect_map.find(name);
    ER_ASSERT(m_config.detect_map.end() != detection,
              "Detection %s not found in configured map: cannot disable",
              name.c_str());
    detection->second.enabled = false;
  }
  void enable(const std::string& name) {
    auto detection = m_config.detect_map.find(name);
    ER_ASSERT(m_config.detect_map.end() != detection,
              "Detection %s not found in configured map: cannot enable",
              name.c_str());
    detection->second.enabled = true;
  }

  void config_update(
      const chsensors::config::env_sensor_config* const config) {
    m_config = *config;
  }


 protected:
  const chsensors::config::env_sensor_config* config(void) const {
    return &m_config;
  }

 private:
  /* clang-format off */
  chsensors::config::env_sensor_config m_config;
  /* clang-format on */
};

NS_END(sensors, hal, cosm);
