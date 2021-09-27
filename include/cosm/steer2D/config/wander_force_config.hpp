/**
 * \file wander_force_config.hpp
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

#ifndef INCLUDE_COSM_STEER2D_CONFIG_WANDER_FORCE_CONFIG_HPP_
#define INCLUDE_COSM_STEER2D_CONFIG_WANDER_FORCE_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/base_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \struct wander_force_config
 * \ingroup steer2D config
 *
 * \brief Configuration for the wander force, as described in \todo ref here.
 */
struct wander_force_config final : public rconfig::base_config {
  /**
   * \brief How often to apply the wander force. 1 = apply every time it is
   * asked. > 1 only apply every nth time it is asked. Depending on the
   * kinematics of the entity in question, applying the wander force every time
   * may not produce the desired level of exploration, because +/- random
   * perturbations to the velocity vector will, by definition, add up to 0 in
   * the long run (unless a normal distribution is used).
   */
  uint interval{0};

  /**
   * \brief Maximum value of the wander force.
   */
  double max{0.0};

  /**
   * \brief Distance of the center of the circle used to calculate the wander
   * force from the entity. Larger value = higher magnitude of wander force
   * vector.
   */
  double circle_distance{0};

  /**
   * \brief Radius of the center of the circle used to calculate the wander
   * force. Large value = higher magnitude of wander force vector.
   *
   */
  double circle_radius{0};

  /**
   * \brief Defines the range [0, value] from which a random value will be
   * sampled uniformly each timestep in order to determine how much +/- angle
   * deviation from a straight line trajectory should be applied. High value =
   * higher average per-timestep deviation/more visibly random wandering.y
   */
  double max_angle_delta{0};

  /**
   * \brief If \c TRUE, then a normal distribution is used instead of a uniform
   * distribution to generate the wander angle.
   */
  bool normal_dist{false};
};

NS_END(config, steer2D, cosm);

#endif /* INCLUDE_COSM_STEER2D_CONFIG_WANDER_FORCE_CONFIG_HPP_ */
