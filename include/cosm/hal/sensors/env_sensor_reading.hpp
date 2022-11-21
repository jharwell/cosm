/**
 * \file env_sensor_reading.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct env_sensor_reading {
  env_sensor_reading(void) = default;
  explicit env_sensor_reading(double v) noexcept : value(v) {}

  double value{0};
};

} /* namespace cosm::hal::sensors */
