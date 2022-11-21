/**
 * \file constant_speed_force.cpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/flocking/constant_speed_force.hpp"

#include <numeric>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::flocking {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
constant_speed_force::constant_speed_force(const config::constant_speed_force_config* config)
    : mc_max(config->max) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d constant_speed_force::operator()(
    const boid& agent,
    const std::vector<rmath::vector2d>& neighbors) const {
  auto avg_vel = std::accumulate(std::begin(neighbors),
                                 std::end(neighbors),
                                 rmath::vector2d()) / neighbors.size();

  auto agent_vel = agent.odometry().twist.linear.to_2D();
  auto sum =  avg_vel + agent_vel;
  auto term = sum * (1 - sum.length())/ (1.0 + std::pow(sum.length(), kBETA));

  return term * mc_max;
} /* operator()() */

} /* namespace cosm::apf2D::flocking */
