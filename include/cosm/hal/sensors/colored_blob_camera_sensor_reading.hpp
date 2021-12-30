/**
 * \file colored_blob_camera_sensor_reading.hpp
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

#ifndef INCLUDE_COSM_HAL_SENSORS_COLORED_BLOB_CAMERA_SENSOR_READING_HPP_
#define INCLUDE_COSM_HAL_SENSORS_COLORED_BLOB_CAMERA_SENSOR_READING_HPP_

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

#endif /* INCLUDE_COSM_HAL_SENSORS_COLORED_BLOB_CAMERA_SENSOR_READING_HPP_ */
