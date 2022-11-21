/**
 * \file alignment_force.hpp
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
#include "cosm/apf2D/flocking/config/alignment_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::flocking {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class alignment_force
 * \ingroup apf2D flocking
 *
 * \brief A force pulling an agent towards a single other agent, or towards the
 * centroid of the positions of a set of agents.
 *
 * From \cite FLOCK:Bagarti2018-stochfov.
 */
class alignment_force {
 public:
  explicit alignment_force(const config::alignment_force_config* config);

   /**
   * \brief Calculate the force.
   *
   * \param agent The current agent.
   *
   * \param neighbors The velocities of the neighbors of \p agent.
   *
   * \return The heading correction the agent should take to stay aligned with
   *         the average position of \p neighbors.
   */

  rmath::vector2d operator()(const boid& agent,
                             const std::vector<rmath::vector2d>& others) const;

 private:
  /* clang-format off */
  const double mc_max;
  /* clang-format on */
};

} /* namespace cosm::apf2D::flocking */
