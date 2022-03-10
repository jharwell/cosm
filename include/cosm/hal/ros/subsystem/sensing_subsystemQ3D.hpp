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
#include <utility>

#include "cosm/hal/subsystem/base_sensing_subsystemQ3D.hpp"

#include "cosm/hal/ros/sensors/lidar_sensor.hpp"
#include "cosm/hal/ros/sensors/odometry_sensor.hpp"
#include "cosm/hal/sensors/proximity_sensor.hpp"
#include "cosm/hal/sensors/env_sensor.hpp"
#include "cosm/hal/ros/sensors/sonar_sensor.hpp"
#include "cosm/hal/ros/sensors/light_sensor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, ros, subsystem);

/*******************************************************************************
 * Macros
 ******************************************************************************/

#if(COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
#define COSM_HAL_ROBOT_SENSOR_TYPES             \
  chros::sensors::light_sensor,                 \
  chros::sensors::lidar_sensor,                 \
  chros::sensors::sonar_sensor,                 \
  chros::sensors::odometry_sensor,              \
  chsensors::proximity_sensor,                  \
  chsensors::env_sensor
#endif

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sensing_subsystemQ3D
 * \ingroup hal subsystem
 *
 * \brief Base sensing subsystem for all sensors used by all ROS robot
 * controllers which actuate in 2D, but can sense in 3D (quasi-3D). It manages
 * an arbitrary number of sensors, depending on which robot the HAL is built
 * for:
 *
 * - \ref chros::sensors::light_sensor
 * - \ref chros::sensors::sonar_sensor
 * - \ref chros::sensors::lidar_sensor
 * - \ref chros::sensors::odometry_sensor
 * - \ref chal::sensors::proximity_sensor
 * - \ref chal::sensors::env_sensor
 */
class sensing_subsystemQ3D :
    public chsubsystem::base_sensing_subsystemQ3D<COSM_HAL_ROBOT_SENSOR_TYPES> {
 public:
  explicit sensing_subsystemQ3D(sensor_map&& sensors)
      : base_sensing_subsystemQ3D(std::move(sensors)) {}

  bool nest_detect(void) const override { return false; }


  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chros::sensors::light_sensor,
                           light);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chros::sensors::light_sensor,
                           light,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chros::sensors::sonar_sensor,
                           sonar);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chros::sensors::sonar_sensor,
                           sonar,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chros::sensors::lidar_sensor,
                           lidar);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chros::sensors::lidar_sensor,
                           lidar,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chros::sensors::odometry_sensor,
                           odometry);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chros::sensors::odometry_sensor,
                           odometry,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chsensors::proximity_sensor,
                           proximity);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chsensors::proximity_sensor,
                           proximity,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chsensors::env_sensor,
                           env);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           chsensors::env_sensor,
                           env,
                           const);
};

NS_END(subsystem, ros, hal, cosm);
