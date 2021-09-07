/**
 * \file avoidance_force.hpp
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

#ifndef INCLUDE_COSM_STEER2D_AVOIDANCE_FORCE_HPP_
#define INCLUDE_COSM_STEER2D_AVOIDANCE_FORCE_HPP_

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
namespace config {
struct avoidance_force_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class avoidance_force
 * \ingroup steer2D
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

NS_END(steer2D, cosm);

#endif /* INCLUDE_COSM_STEER2D_AVOIDANCE_FORCE_HPP_ */
