/**
 * \file saa_names.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/hal/hal.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, argos, subsystem, config, xml);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct saa_names
 * \ingroup hal argos subsystem config xml
 *
 * \brief Collection of sensing and actuation component names that ARGoS expects
 * when asking for handles to components that can be specified for
 * robots. Collected here in a single place in the interest of DRY.
 */
struct saa_names {
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
  static inline const std::string leds_saa = "leds";
  static inline const std::string position_sensor = "positioning";
  static inline const std::string battery_sensor = "battery";
#endif

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT
  static inline const std::string camera_sensor = "colored_blob_omnidirectional_camera";
  static inline const std::string diff_steering_saa = "differential_steering";
  static inline const std::string ir_sensor = "footbot_proximity";
  static inline const std::string light_sensor = "footbot_light";
  static inline const std::string ground_sensor = "footbot_motor_ground";
#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D
  static inline const std::string camera_sensor = "colored_blob_omnidirectional_camera";
  static inline const std::string diff_steering_saa = "differential_steering";
  static inline const std::string ir_sensor = "epuck_proximity";
  static inline const std::string light_sensor = "epuck_light";
  static inline const std::string ground_sensor = "epuck_ground";
#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK
  static inline const std::string diff_steering_saa = "pipuck_differential_drive";
  static inline const std::string ground_sensor = "pipuck_ground";
#endif
};

NS_END(xml, config, subsystem, argos, hal, cosm);
