/**
 * \file seek_force.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/seek_force.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d seek_force::operator()(const boid& agent,
                                       const rmath::vector2d& target) const {
  auto odom = agent.odometry();
  auto pos = odom.pose.position.to_2D();
  auto orientation = rmath::radians(odom.pose.orientation.z());

  /*
   * The angle of the agent's current velocity needs to be 'orientation', NOT
   * the angle of the currently reported velocity vector, because if we are
   * executing a hard turn then the latter won't change, which causes the seek
   * force not to work correctly.
   */
  auto current_vel = rmath::vector2d(odom.twist.linear.to_2D().length(),
                                     orientation);
  auto desired_vel = (target - pos).normalize().scale(mc_max);

  ER_DEBUG("Robot: position=%s,orientation=%s",
           rcppsw::to_string(odom.pose.position.to_2D()).c_str(),
           rcppsw::to_string(orientation).c_str());

  ER_DEBUG("Target=%s@%s [%f]",
           target.to_str().c_str(),
           target.angle().to_str().c_str(),
           target.length());
  ER_DEBUG("Desired velocity vector=%s@%s [%f]",
           desired_vel.to_str().c_str(),
           desired_vel.angle().to_str().c_str(),
           desired_vel.length());

  /*
   * atan2() is discontinuous at angles ~pi! So we wrap the angle to target
   * into [-pi,pi].
   *
   * This used to take the absolute value in order to get something [0, pi],
   * which worked pretty well until COSM#13 (or something else around the time
   * that was merged), after which time robots could not turn left. Removing the
   * abs() around the angle, and using angle instead of -angle in the return
   * vector seems to have done the trick, for now. See COSM#39,RCPPSW#232.
   */
  auto angle = (desired_vel.angle() - orientation).signed_normalize();
  return {mc_max, angle};
} /* operator()() */

} /* namespace cosm::apf2D::nav */
