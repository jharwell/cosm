/**
 * \file env_sensor_impl.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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

#include "cosm/cosm.hpp"

#include "cosm/hal/sensors/config/env_sensor_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_sensor_impl
 * \ingroup hal sensors
 *
 * \brief Interference describing the necessary functionality for robot sensors
 * to implement to be used with \ref env_sensor.
 */
class env_sensor_impl {
 public:
  explicit env_sensor_impl(const chsensors::config::env_sensor_config* const config)
      : m_config(*config) {}

  virtual ~env_sensor_impl(void) = default;

  /* Not copy constructable/assignable by default */
  env_sensor_impl(const env_sensor_impl&) = delete;
  env_sensor_impl& operator=(const env_sensor_impl&) = delete;
  env_sensor_impl(env_sensor_impl&&) = default;
  env_sensor_impl& operator=(env_sensor_impl&&) = default;

  /**
   * \brief Detect the environmental feature represented by \p name.
   */
  virtual bool detect(const std::string& name) const = 0;

  void config_update(
      const chsensors::config::env_sensor_config* const config) {
    m_config = *config;
  }

 protected:
  const chsensors::config::env_sensor_config* config(void) const {
    return &m_config;
  }


};

NS_END(sensors, hal, cosm);
