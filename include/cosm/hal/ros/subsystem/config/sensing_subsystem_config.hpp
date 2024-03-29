/**
 * \file sensing_subsystem_config.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "cosm/hal/sensors/config/proximity_sensor_config.hpp"
#include "cosm/hal/sensors/config/env_sensor_config.hpp"
#include "cosm/hal/ros/sensors/config/sonar_sensor_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::ros::subsystem::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct sensing_subsystem_config
 * \ingroup hal ros subsystem config
 *
 * \brief Hardware-agnostic sensing subsystem configuration for ROS robots.
 */
struct sensing_subsystem_config final : public rconfig::base_config {
  chsensors::config::proximity_sensor_config proximity {};
  chsensors::config::env_sensor_config env {};
  chrsensors::config::sonar_sensor_config sonar {};
};

} /* namespace cosm::hal::ros::subsystem::config */
