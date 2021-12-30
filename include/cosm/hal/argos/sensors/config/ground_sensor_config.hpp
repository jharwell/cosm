/**
 * \file ground_sensor_config.hpp
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

#ifndef INCLUDE_COSM_HAL_ARGOS_SENSORS_CONFIG_GROUND_SENSOR_CONFIG_HPP_
#define INCLUDE_COSM_HAL_ARGOS_SENSORS_CONFIG_GROUND_SENSOR_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>
#include <utility>

#include "rcppsw/config/base_config.hpp"
#include "cosm/hal/argos/sensors/config/ground_sensor_detection_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, argos, sensors, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct ground_sensor_config
 * \ingroup hal argos sensors config
 *
 * \brief Configuration for ground sensors, for robots that have them.
 */
struct ground_sensor_config final : public rconfig::base_config {
  std::map<std::string, ground_sensor_detection_config> detect_map{};
};

NS_END(config, sensors, argos, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ARGOS_SENSORS_CONFIG_GROUND_SENSOR_CONFIG_HPP_ */
