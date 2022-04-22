/**
 * \file seek_force.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"

#include "cosm/cosm.hpp"
#include "cosm/steer2D/boid.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class seek_force
 * \ingroup steer2D
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

NS_END(steer2D, cosm);
