/**
 * \file proximity_sensor_reading.hpp
 *
 * \copyright Copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
  */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct proximity_sensor_reading
 * \ingroup hal sensors
 */
using proximity_sensor_reading = rmath::vector2d;

} /* namespace cosm::hal::sensors */
