/**
 * \file env_sensor.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
