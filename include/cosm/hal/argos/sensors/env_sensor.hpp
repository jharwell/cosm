/**
 * \file env_sensor.hpp
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
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/argos/sensors/ground_sensor.hpp"
#include "cosm/hal/sensors/env_sensor_impl.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, argos, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_sensor
 * \ingroup hal argos sensors
 *
 * \brief Environment sensor wrapper to provide a uniform interface to sensing
 * (roughly) "what is the environment/spatial features of the environment/etc
 * near me" regardless of robot and platform. Provides some additional higher
 * level functionality beyond raw sensor readings too.
 *
 * Supports the following robots:
 *
 * - ARGoS footbot
 * - ARGoS epuck
 * - ARGoS pipuck (stub only)
 */
class env_sensor : public rer::client<env_sensor>,
                   public chsensors::base_sensor<chasensors::ground_sensor>,
                   public chsensors::env_sensor_impl {
 public:
  using ground = chsensors::base_sensor<chasensors::ground_sensor>;

  explicit env_sensor(chasensors::ground_sensor&& sensor);

  /* move only constructible/assignable to work with the saa subsystem */
  env_sensor& operator=(const env_sensor&) = delete;
  env_sensor(const env_sensor&) = delete;
  env_sensor& operator=(env_sensor&& rhs) = default;
  env_sensor(env_sensor&& other) = default;

  /* base_sensor overrides */
  void reset(void) override;
  void enable(void) override;
  void disable(void) override;
  bool is_enabled(void) const override;

  /* env_sensor_impl overrides */
  bool detect(const std::string& name,
              const chsensors::config::env_sensor_detection_config* config) override;
};

NS_END(sensors, argos, hal, cosm);
