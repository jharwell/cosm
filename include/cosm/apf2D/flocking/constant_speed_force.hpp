/**
 * \file constant_speed_force.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "cosm/cosm.hpp"
#include "cosm/apf2D/boid.hpp"
#include "cosm/apf2D/flocking/config/constant_speed_force_config.hpp"
#include "cosm/apf2D/base_force.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::flocking {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class constant_speed_force
 * \ingroup apf2D flocking
 *
 * \brief A force ensuring that an agent and its immediate neighbors maintain a
 * constant average speed.
 *
 * Inspired \cite FLOCK:Bargarti2018-stochfov, but not from there, because the
 * math did not work out, despite what the authors claimed.
 */
class constant_speed_force : public rer::client<constant_speed_force>,
                             public capf2D::base_force {
 public:
  explicit constant_speed_force(const config::constant_speed_force_config* config);

  /**
   * \brief Calculate the force.
   *
   * \param agent The current agent.
   *
   * \param neighbors The velocities of the neighbors of \p agent.
   */
  rmath::vector2d operator()(const boid& agent,
                             const std::vector<rmath::vector2d>& neighbors) const;

 private:

  /* clang-format off */
  const config::constant_speed_force_config mc_config;
  /* clang-format on */
};

} /* namespace cosm::apf2D::flocking */
