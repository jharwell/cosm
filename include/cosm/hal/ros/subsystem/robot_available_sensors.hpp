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
#include "cosm/hal/ros/sensors/lidar_sensor.hpp"
#include "cosm/hal/sensors/odometry_sensor.hpp"
#include "cosm/hal/sensors/proximity_sensor.hpp"
#include "cosm/hal/sensors/env_sensor.hpp"

#if(COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
#include "cosm/hal/ros/sensors/light_sensor.hpp"
#include "cosm/hal/ros/sensors/sonar_sensor.hpp"
#endif

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, ros, subsystem);

/*******************************************************************************
 * Macros
 ******************************************************************************/
#if(COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
#define COSM_HAL_ROBOT_AVAILABLE_SENSORS             \
  chros::sensors::light_sensor,                 \
  chros::sensors::lidar_sensor,                 \
  chros::sensors::sonar_sensor,                 \
  chros::sensors::odometry_sensor,              \
  chsensors::proximity_sensor,                  \
  chsensors::env_sensor
#endif


NS_END(subsystem, ros, hal, cosm);
