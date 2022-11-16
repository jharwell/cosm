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
rmath::vector2d seek_force::operator()(const boid& entity,
                                       const rmath::vector2d& target) const {
  auto odom = entity.odometry();
  rmath::vector2d desired = (target - odom.pose.position.to_2D()).normalize();
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
  auto angle =
      (desired.angle() - odom.twist.linear.to_2D().angle()).signed_normalize();
  return rmath::vector2d(desired.length(), angle).scale(mc_max);
} /* operator()() */

} /* namespace cosm::apf2D::nav */
