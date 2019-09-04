/**
 * @file wander_force.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_STEER2D_WANDER_FORCE_HPP_
#define INCLUDE_COSM_STEER2D_WANDER_FORCE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <random>

#include "rcppsw/rcppsw.hpp"
#include "rcppsw/math/radians.hpp"

#include "cosm/steer2D/boid.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);
namespace config {
struct wander_force_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class wander_force
 * @ingroup cosm steer2D
 *
 * @brief A small random perturbation that can be added to a robot's current
 * velocity in order to make it move randomly throughout the environment. This
 * can be thought of as a directed random walk.
 */
class wander_force {
 public:
  explicit wander_force(const config::wander_force_config* config);

  rmath::vector2d operator()(const boid& entity);

 private:
  /* clang-format off */
  uint                             m_interval;
  int                              m_count{-1};
  bool                             m_use_normal;
  double                           m_max;
  double                           m_circle_distance;
  double                           m_circle_radius;
  double                           m_max_angle_delta;
  double                           m_last_angle{0.0};
  rmath::radians                    m_angle;
  std::default_random_engine       m_rng{};
  std::normal_distribution<double> m_normal_dist;
  std::uniform_real_distribution<> m_uniform_dist;
  /* clang-format on */
};

NS_END(steer2D, cosm);

#endif /* INCLUDE_COSM_STEER2D_WANDER_FORCE_HPP_ */
