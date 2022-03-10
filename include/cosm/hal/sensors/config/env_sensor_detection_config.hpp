/**
 * \file env_sensor_detection_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/range.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, sensors, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct env_sensor_detection_config
 * \ingroup hal sensors config
 *
 * \brief Configuration for robots to detect features of the environment in a
 * simple way.
 */
struct env_sensor_detection_config final : public rconfig::base_config {
  /**
   * \brief What is the range of sensor values (assuming some scalar
   * value/reduction to a scalar) indicating detection of the environmental
   * feature?
   */
  rmath::ranged range{};

  /**
   * \brief If the robot has more than 1 of a given sensor, how many sensors
   * need to agree for successful detection of the environmental feature?
   */
  size_t        consensus{0};
};

NS_END(config, sensors, hal, cosm);
