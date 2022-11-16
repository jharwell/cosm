/**
 * \file colored_blob_camera_sensor_reading.hpp
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

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/utils/color.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct colored_blob_camera_sensor_reading
 * \ingroup hal sensors
 *
 * \brief A camera sensor reading (color, distance in meters) tuple.
 */
struct colored_blob_camera_sensor_reading {
  rmath::vector2d vec{};
  rutils::color color{};
};

NS_END(sensors, hal, cosm);

