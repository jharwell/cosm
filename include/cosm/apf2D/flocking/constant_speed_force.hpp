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
 * From \cite FLOCK:Bargarti2018-stochfov.
 */
class constant_speed_force {
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
  static constexpr const double kBETA = 3.0;

  /* clang-format off */
  const double mc_max;
  /* clang-format on */
};

} /* namespace cosm::apf2D::flocking */
