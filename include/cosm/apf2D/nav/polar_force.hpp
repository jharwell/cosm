/**
 * \file polar_force.hpp
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

#include "cosm/apf2D/boid.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav {

namespace config {
struct polar_force_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class polar_force
 * \ingroup apf2D nav
 *
 * \brief A force radiating from a fixed point in space towards the robot, that
 * always pushes the robot away. Used to add curvature to otherwise straight
 * trajectories.
 */
class polar_force {
 public:
  explicit polar_force(const config::polar_force_config* config);

  rmath::vector2d operator()(const boid& entity,
                             const rmath::vector2d& source) const;

 private:
  const double mc_max;
};

} /* namespace cosm::apf2D::nav */
