/**
 * \file proximity_sensor_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/range.hpp"
#include "rcppsw/math/radians.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::sensors::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct proximity_sensor_config
 * \ingroup hal sensors config
 *
 * \brief Configuration for proximity sensors, for robots which have a sensor
 * which can be used for proximity.
 */
struct proximity_sensor_config final : public rconfig::base_config {
  /*
   * \brief Maximum tolerance for the proximity reading between the robot and
   * the closest obstacle.  The proximity reading is 0 when nothing is detected
   * and grows exponentially to 1 when the obstacle is touching the robot.
   */
  double delta{0.0};

  /**
   * \brief The range in which robots will consider obstacles (i.e., ignore
   * obstacles that are detected behind the robot).
   */
  rmath::range<rmath::radians> fov{rmath::radians(-1.0), rmath::radians(1.0)};

  /**
   * \brief Apply e^{-x} to proximity sensor readings to reduce compress the
   * range to [0,1]. Ignored if COSM is built for ARGoS (all ARGoS robots are
   * hard-coded to do this).
   */
  bool exp_decay{false};

};

} /* namespace cosm::hal::sensors::config */
