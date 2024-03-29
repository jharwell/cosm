/**
 * \file robot_available_sensors.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

#include "cosm/hal/sensors/env_sensor.hpp"
#include "cosm/hal/sensors/proximity_sensor.hpp"
#include "cosm/hal/argos/sensors/light_sensor.hpp"
#include "cosm/hal/sensors/battery_sensor.hpp"

#if defined (COSM_HAL_TARGET_HAS_ODOM_SENSOR)
#include "cosm/hal/sensors/odometry_sensor.hpp"
#endif

#if defined (COSM_HAL_TARGET_HAS_CAMERA_BLOBS_SENSOR)
#include "cosm/hal/argos/sensors/colored_blob_camera_sensor.hpp"
#endif

#if defined (COSM_HAL_TARGET_HAS_WIFI_SENSOR)
#include "cosm/hal/argos/sensors/wifi_sensor.hpp"
#endif


/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::argos::subsystem {

/*******************************************************************************
 * Macros
 ******************************************************************************/
#if COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT
#define COSM_HAL_ROBOT_AVAILABLE_SENSORS                \
    chargos::sensors::wifi_sensor,                      \
    chargos::sensors::light_sensor,                     \
    chsensors::proximity_sensor,                        \
    chargos::sensors::colored_blob_camera_sensor,       \
      chsensors::env_sensor,                            \
      chsensors::battery_sensor,                        \
      chsensors::odometry_sensor

#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D
#define COSM_HAL_ROBOT_AVAILABLE_SENSORS                \
  chargos::sensors::light_sensor,                       \
    chsensors::proximity_sensor,                        \
    chargos::sensors::colored_blob_camera_sensor,       \
    chsensors::env_sensor,                              \
    chsensors::battery_sensor,                          \
    chsensors::odometry_sensor

#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK
#define COSM_HAL_ROBOT_AVAILABLE_SENSORS                \
  chsensors::proximity_sensor,                          \
    chargos::sensors::colored_blob_camera_sensor,       \
    chsensors::env_sensor,                              \
    chsensors::battery_sensor,                          \
    chsensors::odometry_sensor

#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_DRONE
#define COSM_HAL_ROBOT_AVAILABLE_SENSORS            \
  chsensors::env_sensor,                            \
    chargos::sensors::light_sensor,                 \
    chsensors::proximity_sensor,                    \
    chsensors::battery_sensor,                      \
    chsensors::odometry_sensor
#endif

} /* namespace cosm::hal::argos::subsystem */
