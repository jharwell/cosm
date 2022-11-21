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
    : ER_CLIENT_INIT("cosm.flocking.interaction_probability"),
      mc_theta_max(theta_max),
      mc_mean_interaction_dist(mean_interaction_dist) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double interaction_probability::operator()(const capf2D::boid* self,
                                           size_t other_idx,
                                           const std::vector<ckin::odometry>& others) {

  std::map<int, double> w;

  /* calculate w_ij for all agents in the FOV, tracking the sum as you go */
  double sum = 0;
  for (size_t i = 0; i < others.size(); ++i) {
    auto tmp = calc_for_agent(self, others[i]);
    w[i] = tmp;
    sum += tmp;
  } /* for(i..) */

  /* Calculate w_ij for the agent of interest */
  if (rmath::is_equal(sum, 0.0)) {
    return 0.0;
  }
  return  w[other_idx] / sum;
} /* operator()() */

double interaction_probability::calc_for_agent(const capf2D::boid* self,
                                               const ckin::odometry& other) {
  auto self_pos = self->odometry().pose.position.to_2D();
  auto self_vel = self->odometry().twist.linear.to_2D();
  auto other_pos = other.pose.position.to_2D();
  auto other_vel = other.twist.linear.to_2D();

  /* paper says "bearing angle" -> angle to Z axis */
  auto theta_ij = (self_vel - other_vel).angle().unsigned_normalize();

  ER_TRACE("theta_ij: %s", rcppsw::to_string(theta_ij).c_str());

  /*
   * Outside of range (but still inside our FOV, or we would not be in this
   * function).
   */
  if (theta_ij > mc_theta_max) {
    return 0.0;
  }

  auto dist = (self_pos - other_pos).length();
  auto exponent = (std::pow(dist, 2.0) /
                   (2.0 * std::pow(mc_mean_interaction_dist.v(), 2.0)));
  ER_TRACE("Dist to other: %.10e", dist);
  auto exp_term = std::exp(-exponent);
  ER_TRACE("exp_term: %.10e", exp_term);


  auto angle_term = (1 - std::pow(theta_ij.v(), 2.0) /
                     std::pow(mc_theta_max.v(), 2.0));
  ER_TRACE("angle_term: %.10e", angle_term);
  auto w_ij = dist * exp_term * angle_term;
  ER_TRACE("w_ij: %.10e", w_ij);
  return w_ij;
} /* calc_for_agent() */

} /* namespace cosm::flocking */
