/**
 * \file alignment_force.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
    : ER_CLIENT_INIT("cosm.apf2D.flocking.alignment_force"),
      mc_max(config->max) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d alignment_force::operator()(
    const boid& agent,
    const std::vector<rmath::vector2d>& neighbors) const {

  auto odom = agent.odometry();
  auto pos = odom.pose.position.to_2D();
  auto orientation = rmath::radians(odom.pose.orientation.z());

  auto avg_vel = std::accumulate(std::begin(neighbors),
                                 std::end(neighbors),
                                 rmath::vector2d()) / neighbors.size();
  ER_DEBUG("Calculated velocity centroid from %zu neighbors: %s@%s",
           neighbors.size(),
           rcppsw::to_string(avg_vel).c_str(),
           rcppsw::to_string(avg_vel.angle()).c_str());

  /*
   * The angle of the agent's current velocity needs to be 'orientation', NOT
   * the angle of the currently reported velocity vector, because if we are
   * executing a hard turn then the latter won't change, which causes the
   * alignment force not to work correctly.
   */
  auto agent_vel = rmath::vector2d(odom.twist.linear.to_2D().length(),
                                   orientation);
  auto desired = (avg_vel - agent_vel);

  ER_DEBUG("Self velocity=%s@%s, desired=%s@%s",
           rcppsw::to_string(agent_vel).c_str(),
           rcppsw::to_string(agent_vel.angle()).c_str(),
           rcppsw::to_string(desired).c_str(),
           rcppsw::to_string(desired.angle()).c_str());

  /*
   * atan2() is discontinuous at angles ~pi so we wrap the angle to target
   * into [-pi,pi]. See also COSM#39.
   */
  /* auto angle = (desired.angle() - orientation).signed_normalize(); */
  auto angle = (avg_vel.angle() - orientation).signed_normalize();
  return { mc_max, angle};
} /* operator()() */

} /* namespace cosm::apf2D::flocking */
