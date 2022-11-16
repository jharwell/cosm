/**
 * \file seek_force.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class seek_force
 * \ingroup apf2D nav
 *
 * \brief A force pulling the robot to a target and then through the target
 * (i.e. the robot does not slow down to "arrive").
 */
class seek_force {
 public:
  explicit seek_force(double max) : mc_max(max) {}

  rmath::vector2d operator()(const boid& entity,
                             const rmath::vector2d& target) const;

 private:
  /* clang-format off */
  const double mc_max;
  /* clang-format on */
};

} /* namespace cosm::apf2D::nav */
