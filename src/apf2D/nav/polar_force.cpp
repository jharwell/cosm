/**
 * \file polar_force.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/polar_force.hpp"

#include "cosm/apf2D/nav/config/polar_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
polar_force::polar_force(const config::polar_force_config* const config)
    : mc_max(config->max) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d polar_force::operator()(const boid& entity,
                                        const rmath::vector2d& source) const {
  auto odom = entity.odometry();
  auto vel = odom.pose.position.to_2D();

  auto to_src = (odom.pose.position.to_2D() - source).normalize();
  rmath::vector2d orthogonal(-to_src.y(), to_src.x());

  /* ensure velocity is always non-zero */
  vel = make_vel_floor(vel);

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
  auto angle = (orthogonal.angle() - vel.angle()).signed_normalize();
  return rmath::vector2d(orthogonal.length(), angle).scale(mc_max);
} /* operator()() */

} /* namespace cosm::apf2D::nav */
