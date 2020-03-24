/**
 * \file saa_xml_names.hpp
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

#ifndef INCLUDE_COSM_ROBOTS_FOOTBOT_CONFIG_SAA_XML_NAMES_HPP_
#define INCLUDE_COSM_ROBOTS_FOOTBOT_CONFIG_SAA_XML_NAMES_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"
#include "cosm/hal/hal.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, robots, footbot, config);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct saa_xml_names
 * \ingroup robots footbot config
 *
 * \brief Collection of names of sensing and actuation component names that
 * ARGoS expects when asking for handles to robot components that can be
 * specified for the foot-bot robot. Collected here in a single place in the
 * interest of DRY.
 */
struct saa_xml_names {
  static constexpr char diff_steering_saa[] = "differential_steering";
  static constexpr char leds_saa[] = "leds";
  static constexpr char rab_saa[] = "range_and_bearing";
  static constexpr char prox_sensor[] = "footbot_proximity";
  static constexpr char position_sensor[] = "positioning";
  static constexpr char camera_sensor[] = "colored_blob_omnidirectional_camera";
  static constexpr char light_sensor[] = "footbot_light";
  static constexpr char ground_sensor[] = "footbot_motor_ground";
  static constexpr char battery_sensor[] = "battery";
};

NS_END(config, footbot, robots, cosm);

#endif /* INCLUDE_COSM_ROBOTS_FOOTBOT_CONFIG_SAA_XML_NAMES_HPP_ */
