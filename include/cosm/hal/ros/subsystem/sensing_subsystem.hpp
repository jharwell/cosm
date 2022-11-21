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
#include "cosm/hal/ros/subsystem/robot_available_sensors.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::ros::subsystem {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sensing_subsystem
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
 * - \ref chsensors::proximity_sensor
 * - \ref chsensors::env_sensor
 * - \ref chsensors::battery_sensor
 */
class sensing_subsystem :
    public chsubsystem::base_sensing_subsystem<COSM_HAL_ROBOT_AVAILABLE_SENSORS> {
 public:
  explicit sensing_subsystem(sensor_map&& sensors)
      : base_sensing_subsystem(std::move(sensors)) {}


  COSM_HAL_SENSOR_ACCESSOR(chros::sensors::light_sensor, light);
  COSM_HAL_SENSOR_ACCESSOR(chros::sensors::light_sensor, light, const);

  COSM_HAL_SENSOR_ACCESSOR(chros::sensors::sonar_sensor, sonar);
  COSM_HAL_SENSOR_ACCESSOR(chros::sensors::sonar_sensor, sonar, const);

  COSM_HAL_SENSOR_ACCESSOR(chros::sensors::lidar_sensor, lidar);
  COSM_HAL_SENSOR_ACCESSOR(chros::sensors::lidar_sensor, lidar, const);

  COSM_HAL_SENSOR_ACCESSOR(chsensors::battery_sensor, battery);
  COSM_HAL_SENSOR_ACCESSOR(chsensors::battery_sensor, battery, const);

  COSM_HAL_SENSOR_ACCESSOR(chros::sensors::odometry_sensor, odometry);
  COSM_HAL_SENSOR_ACCESSOR(chros::sensors::odometry_sensor, odometry, const);

  COSM_HAL_SENSOR_ACCESSOR(chsensors::proximity_sensor, proximity);
  COSM_HAL_SENSOR_ACCESSOR(chsensors::proximity_sensor, proximity, const);

  COSM_HAL_SENSOR_ACCESSOR(chsensors::env_sensor, env);
  COSM_HAL_SENSOR_ACCESSOR(chsensors::env_sensor, env, const);
};

} /* namespace cosm::hal::ros::subsystem */
