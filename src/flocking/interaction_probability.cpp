/**
 * \file interaction_probability.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/flocking/interaction_probability.hpp"

#include <map>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::flocking {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
interaction_probability::interaction_probability(const rmath::radians& theta_max,
                                                 const rspatial::euclidean_dist& mean_interaction_dist)
    : mc_theta_max(theta_max),
      mc_mean_interaction_dist(mean_interaction_dist) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::probability interaction_probability::operator()(
    const rmath::vector2d& self_pos,
    const rmath::vector2d& self_vel,
    size_t other_idx,
    const std::vector<rmath::vector2d>& fov_agents) {

  std::map<int, rmath::probability> w;

  /* calculate w_ij for all agents in the FOV, tracking the sum as you go */
  rmath::probability sum(0);
  for (size_t i = 0; i < fov_agents.size(); ++i) {
    auto tmp = calc_for_agent(self_pos, self_vel, fov_agents[i]);
    w[i] = tmp;
    sum += tmp;
  } /* for(i..) */

  /* Calculate w_ij for the agent of interest */
  return rmath::probability(w[other_idx] / sum);
} /* operator()() */

rmath::probability interaction_probability::calc_for_agent(
    const rmath::vector2d& pos,
    const rmath::vector2d& vel,
    const rmath::vector2d fov_agent_pos) {
  /* paper says "bearing angle" -> angle to Z axis */
  auto heading_diff = pos - fov_agent_pos;
  auto theta_ij = (vel.angle() - heading_diff.angle()).unsigned_normalize();

  /*
   * Outside of range (but still inside our FOV, or we would not be in this
   * function).
   */
  if (theta_ij > mc_theta_max) {
    return rmath::probability(0.0);
  }

  auto dist = (pos - fov_agent_pos).length();
  auto exponent = (std::pow(dist, 2.0) / (2.0 * mc_mean_interaction_dist.v()));
  auto exp_term = std::exp(-exponent);


  auto angle_term = (1 - std::pow(theta_ij.v(), 2.0) /
                     std::pow(mc_theta_max.v(), 2.0));

  auto w_ij = rmath::probability(dist * exp_term * angle_term);
  return w_ij;
} /* calc_for_agent() */

} /* namespace cosm::flocking */
