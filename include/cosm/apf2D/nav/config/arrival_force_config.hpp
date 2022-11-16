/**
 * \file arrival_force_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/base_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config {

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct arrival_force_config
 * \ingroup apf2D nav config
 *
 * \brief Configuration for the arrival force, as described in \ref
 * arrival_force.
 */
struct arrival_force_config final : public rconfig::base_config {
  /**
   * Maximum value for the force.
   */
  double max{0};

  /**
   * The speed that entities entering the slowing radius will slow down to
   * (linear ramp down).
   */
  double slowing_speed_min{0};

  /**
   * The radius around the object inside which the entity should begin to slow
   * down, so as to not overshoot the target.
   */
  double slowing_radius{0};
};

} /* namespace cosm::apf2D::nav::config */
