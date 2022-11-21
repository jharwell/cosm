/**
 * \file constant_speed_force.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/flocking/constant_speed_force.hpp"

#include <numeric>
#include <algorithm>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::flocking {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
constant_speed_force::constant_speed_force(const config::constant_speed_force_config* config)
    : ER_CLIENT_INIT("cosm.apf2D.flocking.constant_speed_force"),
      mc_config(*config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d constant_speed_force::operator()(
    const boid& agent,
    const std::vector<rmath::vector2d>& neighbors) const {
  if (rmath::is_equal(0.0, mc_config.max)) {
    return {};
  }
  auto avg_vel = std::accumulate(std::begin(neighbors),
                                 std::end(neighbors),
                                 rmath::vector2d()) / neighbors.size();

  auto agent_vel = agent.odometry().twist.linear.to_2D();
  auto joint_vel =  avg_vel + agent_vel;
  auto orientation = rmath::radians(agent.odometry().pose.orientation.z());

  /*
   * This sigmoid vanishes when the joint velocity approaches the setpoint, and
   * drives it back to the setpoint if it gets too large/small.
   */
  auto factor = std::exp(mc_config.speed_setpoint -
                         (joint_vel.length() / mc_config.speed_setpoint));
  factor = std::clamp(factor, -mc_config.max, mc_config.max);

  ER_DEBUG("Self speed=%f,neighbors avg_speed=%f,factor=%f",
           agent_vel.length(),
           avg_vel.length(),
           factor);

  /*
   * This is the UPDATE to our current heading as a result of applying this
   * force, so no change in direction/only a change in speed.
   */
  return {factor, rmath::radians::kZERO};
} /* operator()() */

} /* namespace cosm::apf2D::flocking */
