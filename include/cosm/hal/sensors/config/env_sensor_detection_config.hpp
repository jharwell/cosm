/**
 * \file env_sensor_detection_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/range.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::sensors::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct env_sensor_detection_config
 * \ingroup hal sensors config
 *
 * \brief Configuration for robots to detect features of the environment in a
 * simple way.
 */
struct env_sensor_detection_config final : public rconfig::base_config {
  /**
   * \brief What is the range of sensor values (assuming some scalar
   * value/reduction to a scalar) indicating detection of the environmental
   * feature?
   */
  rmath::ranged range{};

  /**
   * \brief If the robot has more than 1 of a given sensor, how many sensors
   * need to agree for successful detection of the environmental feature?
   */
  size_t        consensus{0};

  /**
   * \brief Is detection of this environmental feature enabled?
   */
  bool enabled{true};
};

} /* namespace cosm::hal::sensors::config */
