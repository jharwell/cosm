/**
 * \file sensing_subsystemQ3D.hpp
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
#include "cosm/hal/subsystem/base_sensing_subsystemQ3D.hpp"

#include "cosm/hal/argos/subsystem/robot_available_sensors.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, argos, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sensing_subsystemQ3D
 * \ingroup hal subsystem
 *
 * \brief Base sensing subsystem for all sensors used by all ARGoS robot
 * controllers which actuate in 2D, but can sense in 3D (quasi-3D). It uses a
 * \ref hal::sensors::position_sensor for positioning information, and manages
 * any number of additional sensors, depending on which robot the HAL is built
 * for:
 *
 * - \ref chargos::sensors::colored_blob_camera_sensor
 * - \ref chargos::sensors::light_sensor
 * - \ref chargos::sensors::battery_sensor
 * - \ref chal::sensors::odometry_sensor
 * - \ref chargos::sensors::wifi_sensor
 * - \ref chal::sensors::proximity_sensor
 * - \ref chal::sensors::env_sensor
 */
class sensing_subsystemQ3D :
    public chsubsystem::base_sensing_subsystemQ3D<COSM_HAL_ROBOT_AVAILABLE_SENSORS> {
 public:
  explicit sensing_subsystemQ3D(sensor_map&& sensors)
      : base_sensing_subsystemQ3D(std::move(sensors)) {}

  bool nest_detect(void) const override {
    return env()->detect(chal::sensors::env_sensor::kNestTarget);
  }

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chargos::sensors::colored_blob_camera_sensor,
                           blobs);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chargos::sensors::colored_blob_camera_sensor,
                           blobs,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chargos::sensors::light_sensor,
                           light);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chargos::sensors::light_sensor,
                           light,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chsensors::proximity_sensor,
                           proximity);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chsensors::proximity_sensor,
                           proximity,
                           const);


  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chal::sensors::env_sensor,
                           env);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chal::sensors::env_sensor,
                           env,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chargos::sensors::battery_sensor,
                           battery);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chargos::sensors::battery_sensor,
                           battery,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chal::sensors::odometry_sensor,
                           odometry);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chal::sensors::odometry_sensor,
                           odometry,
                           const);
};

NS_END(subsystem, argos, hal, cosm);

