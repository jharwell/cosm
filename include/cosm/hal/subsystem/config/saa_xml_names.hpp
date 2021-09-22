/**
 * \file saa_xml_names.hpp
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

#ifndef INCLUDE_COSM_HAL_SUBSYSTEM_CONFIG_SAA_XML_NAMES_HPP_
#define INCLUDE_COSM_HAL_SUBSYSTEM_CONFIG_SAA_XML_NAMES_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"
#include "cosm/hal/hal.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, subsystem, config);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct saa_xml_names
 * \ingroup hal subsystem config
 *
 * \brief Collection of names of sensing and actuation component names that
 * ARGoS expects when asking for handles to robot components that can be
 * specified for robots. Collected here in a single place in the interest of
 * DRY.
 */
struct saa_xml_names {
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
  static inline const std::string leds_saa = "leds";
  static inline const std::string position_sensor = "positioning";
#endif

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT
  static inline const std::string camera_sensor = "colored_blob_omnidirectional_camera";
  static inline const std::string diff_steering_saa = "differential_steering";
  static inline const std::string prox_sensor = "footbot_proximity";
  static inline const std::string light_sensor = "footbot_light";
  static inline const std::string ground_sensor = "footbot_motor_ground";
#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D
  static inline const std::string camera_sensor = "colored_blob_omnidirectional_camera";
  static inline const std::string diff_steering_saa = "differential_steering";
  static inline const std::string prox_sensor = "epuck_proximity";
  static inline const std::string light_sensor = "epuck_light";
  static inline const std::string ground_sensor = "epuck_ground";
#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK
  static inline const std::string diff_steering_saa = "pipuck_differential_drive";
  static inline const std::string ground_sensor = "pipuck_ground";
#endif
};

NS_END(config, subsystem, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SUBSYSTEM_CONFIG_SAA_XML_NAMES_HPP_ */
