/**
 * \file env_sensor.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/ros/sensors/env_sensor.hpp"

#include "cosm/hal/sensors/env_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
env_sensor::env_sensor(chrsensors::sonar_sensor&& sonar_sensor,
                       chrsensors::light_sensor&& light_sensor)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.env_sensor"),
      sonar(std::move(sonar_sensor)),
      light(std::move(light_sensor)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void env_sensor::reset(void) {
  sonar::decoratee().reset();
  light::decoratee().reset();
}
void env_sensor::enable(void) {
  sonar::decoratee().enable();
  light::decoratee().enable();
}
void env_sensor::disable(void) {
  sonar::decoratee().disable();
  light::decoratee().disable();
}
bool env_sensor::is_enabled(void) const {
  return sonar::decoratee().is_enabled() && light::decoratee().is_enabled();
}

bool env_sensor::detect(
    const std::string& name,
    const chsensors::config::env_sensor_detection_config* config) {
  size_t sum = 0;
  if (chsensors::env_sensor::kNestTarget == name) {
    for (auto& r : light::decoratee().readings()) {
      sum += static_cast<size_t>(config->range.contains(r.intensity));
    } /* for(&r..) */
    return sum >= config->consensus;
  } else if (chsensors::env_sensor::kBlockTarget == name) {
    for (auto& r : sonar::decoratee().readings()) {
      sum += static_cast<size_t>(config->range.contains(r.value));
    } /* for(&r..) */
    return sum >= config->consensus;
  } else {
    ER_FATAL_SENTINEL("Bad detection: %s", name.c_str());
    return false;
  }
}

NS_END(sensors, ros, hal, cosm);