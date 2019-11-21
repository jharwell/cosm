/**
 * @file proximity_sensor_config.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_SENSORS_CONFIG_PROXIMITY_SENSOR_CONFIG_HPP_
#define INCLUDE_COSM_HAL_SENSORS_CONFIG_PROXIMITY_SENSOR_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/range.hpp"
#include "rcppsw/math/radians.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, sensors, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct proximity_sensor_config
 * @ingroup cosm hal sensors config
 *
 * @brief Configuration for proximity sensors, for robots which have them.
 *
 * Robots are:
 *
 * - ARGoS footbot
 */
struct proximity_sensor_config final : public rconfig::base_config {
  /*
   * Maximum tolerance for the proximity reading between the robot and the
   * closest obstacle.  The proximity reading is 0 when nothing is detected and
   * grows exponentially to 1 when the obstacle is touching the robot.
   */
  double delta{0.0};

  rmath::range<rmath::radians> fov{rmath::radians(-1.0), rmath::radians(1.0)};
};

NS_END(config, sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_CONFIG_PROXIMITY_SENSOR_CONFIG_HPP_ */
