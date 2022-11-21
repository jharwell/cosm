/**
 * \file alignment_force.cpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/flocking/alignment_force.hpp"

#include <numeric>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::flocking {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
alignment_force::alignment_force(const config::alignment_force_config* config)
    : mc_max(config->max) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d alignment_force::operator()(
    const boid& agent,
    const std::vector<rmath::vector2d>& neighbors) const {
  auto avg_vel = std::accumulate(std::begin(neighbors),
                             std::end(neighbors),
                             rmath::vector2d()) / neighbors.size();

  auto vel = agent.odometry().twist.linear.to_2D();
  return (avg_vel - vel) * mc_max;
} /* operator()() */

} /* namespace cosm::apf2D::flocking */
