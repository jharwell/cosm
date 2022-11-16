/**
 * \file avoidance_force.hpp
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

#include "cosm/cosm.hpp"
#include "cosm/apf2D/boid.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav {

namespace config {
struct avoidance_force_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class avoidance_force
 * \ingroup apf2D nav
 *
 * \brief A force pushing the robot away from perceived obstacles. Only active
 * when threatening obstacles are detected.
 */
class avoidance_force {
 public:
  explicit avoidance_force(const config::avoidance_force_config* config);

  /**
   * \brief Calculate the avoidance force that should be applied to the
   * robot. Avoidance force will point from the robot away from the cloest
   * obstacle.
   *
   * \param closest The closest known obstacle to the robot.
   */
  rmath::vector2d operator()(const boid&, const rmath::vector2d& closest) const;

 private:
  /* clang-format off */
  const double mc_max;
  /* clang-format on */
};

} /* namespace cosm::apf2D::nav */
