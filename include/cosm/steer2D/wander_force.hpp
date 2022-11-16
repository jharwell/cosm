/**
 * \file wander_force.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/rcppsw.hpp"

#include "cosm/steer2D/base_bias_angle_generator.hpp"
#include "cosm/steer2D/boid.hpp"
#include "cosm/steer2D/config/wander_force_config.hpp"

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
