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

#include "cosm/steer2D/boid.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);
namespace config {
struct polar_force_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class polar_force
 * \ingroup steer2D
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

NS_END(steer2D, cosm);
