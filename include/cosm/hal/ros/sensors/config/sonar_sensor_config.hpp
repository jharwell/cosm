/**
 * \file sonar_sensor_config.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct sonar_sensor_config
 * \ingroup hal ros sensors config
 *
 * \brief Configuration for HC-SR04 ultrasonic sensors.
 */
struct sonar_sensor_config final : public rconfig::base_config {
  /**
   * \brief The pin the "ping" is sent on.
   */
  int trigger_pin{-1};

  /**
   * \brief The pin the "ping" echo data is received from.
   */
  int echo_pin{-1};
};

NS_END(config, sensors, ros, hal, cosm);
