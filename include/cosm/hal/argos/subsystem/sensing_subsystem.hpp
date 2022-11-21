/**
 * \file sensing_subsystem.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>

#include "cosm/hal/subsystem/base_sensing_subsystem.hpp"

#include "cosm/hal/argos/subsystem/robot_available_sensors.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::argos::subsystem {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sensing_subsystem
 * \ingroup hal subsystem
 *
 * \brief Base sensing subsystem for all sensors used by all ARGoS robot
 * controllers.
 *
 * It uses a \ref hal::sensors::position_sensor for positioning information, and
 * manages any number of additional sensors, depending on which robot the HAL is
 * built for:
 *
 * - \ref chargos::sensors::colored_blob_camera_sensor
 * - \ref chargos::sensors::light_sensor
 * - \ref chsensors::battery_sensor
 * - \ref chsensors::odometry_sensor
 * - \ref chargos::sensors::wifi_sensor
 * - \ref chsensors::proximity_sensor
 * - \ref chsensors::env_sensor
 */
class sensing_subsystem :
    public chsubsystem::base_sensing_subsystem<COSM_HAL_ROBOT_AVAILABLE_SENSORS> {
 public:
  explicit sensing_subsystem(sensor_map&& sensors)
      : base_sensing_subsystem(std::move(sensors)) {}

#if defined(COSM_HAL_TARGET_HAS_BLOB_CAMERA_SENSOR)
  COSM_HAL_SENSOR_ACCESSOR(chargos::sensors::colored_blob_camera_sensor, blobs);
  COSM_HAL_SENSOR_ACCESSOR(chargos::sensors::colored_blob_camera_sensor, blobs, const);
#endif

  COSM_HAL_SENSOR_ACCESSOR(chargos::sensors::light_sensor, light);
  COSM_HAL_SENSOR_ACCESSOR(chargos::sensors::light_sensor, light, const);

  COSM_HAL_SENSOR_ACCESSOR(chsensors::proximity_sensor, proximity);
  COSM_HAL_SENSOR_ACCESSOR(chsensors::proximity_sensor, proximity, const);

  COSM_HAL_SENSOR_ACCESSOR(chsensors::env_sensor, env);
  COSM_HAL_SENSOR_ACCESSOR(chsensors::env_sensor, env, const);

  COSM_HAL_SENSOR_ACCESSOR(chsensors::battery_sensor, battery);
  COSM_HAL_SENSOR_ACCESSOR(chsensors::battery_sensor, battery, const);

#if defined(COSM_HAL_TARGET_HAS_ODOM_SENSOR)
  COSM_HAL_SENSOR_ACCESSOR(chsensors::odometry_sensor, odometry);
  COSM_HAL_SENSOR_ACCESSOR(chsensors::odometry_sensor, odometry, const);
#endif
};

} /* namespace cosm::hal::argos::subsystem */
