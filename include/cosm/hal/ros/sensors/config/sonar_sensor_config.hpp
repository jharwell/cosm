/**
 * \file sonar_sensor_config.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::ros::sensors::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct sonar_sensor_config
 * \ingroup hal ros sensors config
 *
 * \brief Configuration for HC-SR04 ultrasonic sensors.
 */
struct sonar_sensor_config final : public rconfig::base_config {
  /**
   * \brief The pin the "ping" is sent on.
   */
  int trigger_pin{-1};

  /**
   * \brief The pin the "ping" echo data is received from.
   */
  int echo_pin{-1};
};

} /* namespace cosm::hal::ros::sensors::config */
