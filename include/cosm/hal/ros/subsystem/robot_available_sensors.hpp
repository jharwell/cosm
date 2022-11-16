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
#include "cosm/hal/ros/sensors/lidar_sensor.hpp"
#include "cosm/hal/sensors/odometry_sensor.hpp"
#include "cosm/hal/sensors/proximity_sensor.hpp"
#include "cosm/hal/sensors/env_sensor.hpp"
#include "cosm/hal/sensors/battery_sensor.hpp"

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
  chros::sensors::light_sensor,                      \
    chros::sensors::lidar_sensor,                    \
    chsensors::battery_sensor,                       \
    chros::sensors::sonar_sensor,                    \
    chros::sensors::odometry_sensor,                 \
    chsensors::proximity_sensor,                     \
    chsensors::env_sensor
#endif


NS_END(subsystem, ros, hal, cosm);
