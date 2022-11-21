/**
 * \file interaction_probability.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License Identifier: MIT
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
double interaction_probability::operator()(const rmath::vector2d& self_pos,
                                           const rmath::vector2d& self_vel,
                                           size_t other_idx,
                                           const capf2D::boid_vectorro& others) {

  std::map<int, double> w;

  /* calculate w_ij for all agents in the FOV, tracking the sum as you go */
  double sum(0);
  for (size_t i = 0; i < others.size(); ++i) {
    auto tmp = calc_for_agent(self_pos, self_vel, others[i]);
    w[i] = tmp;
    sum += tmp;
  } /* for(i..) */

  /* Calculate w_ij for the agent of interest */
  return  w[other_idx] / sum;
} /* operator()() */

double interaction_probability::calc_for_agent(
    const rmath::vector2d& pos,
    const rmath::vector2d& vel,
    const capf2D::boid* other) {
  auto other_pos = other->odometry().pose.position.to_2D();
  auto other_vel = other->odometry().twist.linear.to_2D();

  /* paper says "bearing angle" -> angle to Z axis */
  auto heading_diff = (vel - other_vel).angle();
  auto theta_ij = (vel.angle() - heading_diff).unsigned_normalize();

  /*
   * Outside of range (but still inside our FOV, or we would not be in this
   * function).
   */
  if (theta_ij > mc_theta_max) {
    return 0.0;
  }

  auto dist = (pos - other_pos).length();
  auto exponent = (std::pow(dist, 2.0) / (2.0 * mc_mean_interaction_dist.v()));
  auto exp_term = std::exp(-exponent);


  auto angle_term = (1 - std::pow(theta_ij.v(), 2.0) /
                     std::pow(mc_theta_max.v(), 2.0));

  auto w_ij = dist * exp_term * angle_term;
  return w_ij;
} /* calc_for_agent() */

} /* namespace cosm::flocking */
