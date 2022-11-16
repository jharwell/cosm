/**
 * \file arrival_force.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/steer2D/arrival_force.hpp"

#include "cosm/steer2D/config/arrival_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arrival_force::arrival_force(const config::arrival_force_config* const config)
    : ER_CLIENT_INIT("cosm.steer2D.arrival_force"),
      mc_max(config->max),
      mc_slowing_speed_min(config->slowing_speed_min),
      mc_slowing_radius(config->slowing_radius) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d arrival_force::operator()(const boid& entity,
                                          const rmath::vector2d& target) {
  auto odom = entity.odometry();
  rmath::vector2d desired = target - odom.pose.position.to_2D();
  double distance = desired.length();

  ER_DEBUG("Robot: position=%s,orientation=%s",
           rcppsw::to_string(odom.pose.position.to_2D()).c_str(),
           rcppsw::to_string(odom.pose.orientation.z()).c_str());

  ER_DEBUG("Target=%s@%s [%f]",
           target.to_str().c_str(),
           target.angle().to_str().c_str(),
           target.length());
  ER_DEBUG("Vector to target=%s@%s [%f]",
           desired.to_str().c_str(),
           desired.angle().to_str().c_str(),
           desired.length());

  desired.normalize();
  if (distance <= mc_slowing_radius) {
    m_within_slowing_radius = true;
    desired.scale(
        std::max(mc_slowing_speed_min, mc_max * distance / mc_slowing_radius));
  } else {
    m_within_slowing_radius = false;
    desired.scale(mc_max);
  }
  /*
   * atan2() is discontinuous at angles ~pi! so we wrap the angle to target
   * into [-pi,pi].
   *
   * This used to take the absolute value in order to get something [0, pi],
   * which worked pretty well until COSM#13 (or something else around the time
   * that was merged), after which time robots could not turn left. Removing the
   * abs() around the angle, and using angle instead of -angle in the return
   * vector seems to have done the trick, for now. See COSM#39,RCPPSW#232.
   */
  auto orientation = rmath::radians(odom.pose.orientation.z());
  auto angle = (desired.angle() - orientation).signed_normalize();
  return { desired.length(), angle };
} /* operator()() */

NS_END(steer2D, cosm);
