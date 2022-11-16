/**
 * \file light_sensor_reading.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/math/radians.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct light_sensor_reading
 * \ingroup hal sensors
 *
 * \brief A light sensor reading (value, angle) pair.
 *
 * The first argument is the value of the sensor, and the second argument is the
 * angle of the sensor on the robot in relation to the positive x axis.
 */
struct light_sensor_reading {
  double intensity;
  rmath::radians angle;
};

NS_END(sensors, hal, cosm);
