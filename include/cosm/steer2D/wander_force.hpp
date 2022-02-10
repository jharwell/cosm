/**
 * \file wander_force.hpp
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
#include <memory>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/rcppsw.hpp"

#include "cosm/steer2D/boid.hpp"
#include "cosm/steer2D/config/wander_force_config.hpp"
#include "cosm/steer2D/base_bias_angle_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class wander_force
 * \ingroup steer2D
 *
 * \brief A small random perturbation that can be added to a robot's current
 * velocity in order to make it move randomly throughout the environment. This
 * can be thought of as a directed random walk.
 */
class wander_force {
 public:
  explicit wander_force(const config::wander_force_config* config);

  rmath::vector2d operator()(const boid& entity, rmath::rng* rng);

 private:
  /* clang-format off */
  const config::wander_force_config          mc_config;

  int                                        m_count{-1};
  rmath::radians                             m_last_angle{0};
  rmath::radians                             m_angle{0};
  std::unique_ptr<base_bias_angle_generator> m_bias_generator;
  /* clang-format on */
};

NS_END(steer2D, cosm);

