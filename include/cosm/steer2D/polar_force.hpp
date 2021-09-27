/**
 * \file polar_force.hpp
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

#ifndef INCLUDE_COSM_STEER2D_POLAR_FORCE_HPP_
#define INCLUDE_COSM_STEER2D_POLAR_FORCE_HPP_

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

#endif /* INCLUDE_COSM_STEER2D_POLAR_FORCE_HPP_ */
