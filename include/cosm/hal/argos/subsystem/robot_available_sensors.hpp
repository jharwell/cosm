/**
 * \file robot_available_sensors.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/hal/argos/sensors/light_sensor.hpp"
#include "cosm/hal/argos/sensors/colored_blob_camera_sensor.hpp"
#include "cosm/hal/sensors/proximity_sensor.hpp"
#include "cosm/hal/argos/sensors/battery_sensor.hpp"
#include "cosm/hal/argos/sensors/wifi_sensor.hpp"
#include "cosm/hal/sensors/odometry_sensor.hpp"
#include "cosm/hal/sensors/env_sensor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, argos, subsystem);

/*******************************************************************************
 * Macros
 ******************************************************************************/
#if COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT
#define COSM_HAL_ROBOT_AVAILABLE_SENSORS                \
  chargos::sensors::battery_sensor,                     \
    chargos::sensors::wifi_sensor,                      \
    chargos::sensors::light_sensor,                     \
    chsensors::proximity_sensor,                        \
    chargos::sensors::colored_blob_camera_sensor,       \
    chsensors::env_sensor,                          \
    chsensors::odometry_sensor

#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D
#define COSM_HAL_ROBOT_AVAILABLE_SENSORS                \
  chargos::sensors::light_sensor,                       \
    chsensors::proximity_sensor,                        \
    chargos::sensors::colored_blob_camera_sensor,       \
    chsensors::env_sensor,                          \
    chsensors::odometry_sensor

#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK
#define COSM_HAL_ROBOT_AVAILABLE_SENSORS                \
  chsensors::proximity_sensor,                          \
    chargos::sensors::colored_blob_camera_sensor,       \
    chsensors::env_sensor,                          \
    chsensors::odometry_sensor
#endif


NS_END(subsystem, argos, hal, cosm);
