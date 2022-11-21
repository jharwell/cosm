/**
 * \file diff_drive_config.hpp
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
#include "rcppsw/math/radians.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::kin2D::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct diff_drive_config
 * \ingroup kin2D config
 *
 * \brief Configuration for differential drive actuator.
 */
struct diff_drive_config final : public rconfig::base_config {
  /**
   * \brief Maximum angle difference between current and new heading that will
   * not trigger a hard (in place) turn.
   */
  rmath::radians soft_turn_max{};

  /**
   * \brief The maximum angular speed. Radians/s.
   */
  double max_angular_speed{0.0};

  /**
   * \brief The maximum linear speed. M/s.
   */
  double max_linear_speed{0.0};
};

} /* namespace cosm::kin2D::config */
