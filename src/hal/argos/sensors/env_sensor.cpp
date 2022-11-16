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
#include "cosm/hal/argos/sensors/env_sensor.hpp"

#include "cosm/hal/sensors/env_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, argos, sensors);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
env_sensor::env_sensor(chasensors::ground_sensor&& sensor)
    : ER_CLIENT_INIT("cosm.hal.argos.sensors.env_sensor"),
      ground(std::move(sensor)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void env_sensor::reset(void) { ground::decoratee().reset(); }
void env_sensor::enable(void) { ground::decoratee().enable(); }

void env_sensor::disable(void) { ground::decoratee().disable(); }
bool env_sensor::is_enabled(void) const {
  return ground::decoratee().is_enabled();
}

bool env_sensor::detect(
    const std::string& name,
    const chsensors::config::env_sensor_detection_config* config) {
  size_t sum = 0;
  if (chsensors::env_sensor::kNestTarget == name) {
    for (auto& r : ground::decoratee().readings()) {
      sum += static_cast<size_t>(config->range.contains(r.value));
    } /* for(&r..) */
    return sum >= config->consensus;
  } else if (chsensors::env_sensor::kBlockTarget == name) {
    for (auto& r : ground::decoratee().readings()) {
      sum += static_cast<size_t>(config->range.contains(r.value));
    } /* for(&r..) */
    return sum >= config->consensus;
  } else if (kCacheTarget == name) {
    for (auto& r : ground::decoratee().readings()) {
      sum += static_cast<size_t>(config->range.contains(r.value));
    } /* for(&r..) */
    return sum >= config->consensus;
  } else {
    ER_FATAL_SENTINEL("Bad detection: %s", name.c_str());
    return false;
  }
}

NS_END(sensors, argos, hal, cosm);
