/**
 * \file wander_force.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/wander_force.hpp"

#include "rcppsw/math/angles.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/apf2D/nav/bias_angle_generator_factory.hpp"
#include "cosm/apf2D/nav/config/wander_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
wander_force::wander_force(const config::wander_force_config* const config)
    : mc_config(*config),
      m_bias_generator(
          bias_angle_generator_factory().create(config->bias_angle.src,
                                                &config->bias_angle)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d wander_force::operator()(const boid& entity, rmath::rng* rng) {
  /*
   * Only actually apply the wander force at the specified cadence. Otherwise
   * random perturbations between [-n, n] will sum to 0 (no net wandering) over
   * time if the perturbations are applied every timestep.
   */
  ++m_count;
  if (m_count % mc_config.interval != 0) {
    return { 0, 0 };
  }

  /* calculate circle center */
  auto odom = entity.odometry();
  auto velocity = odom.twist.linear.to_2D();

  /* ensure velocity is always non-zero */
  velocity = make_vel_floor(velocity);

  auto circle_center = velocity.normalize().scale(mc_config.circle_distance);

  /*
   * Calculate displacement force (the actual wandering) using the wander angle
   * from the PREVIOUS timestep.
   */
  rmath::vector2d displacement(
      mc_config.circle_radius * std::cos((m_angle + velocity.angle()).v()),
      mc_config.circle_radius * std::sin((m_angle + velocity.angle()).v()));

  /*
   * Generate the new bias angle and store it so the next time we calculate it
   * we have a reference point so we can compute the bias correctly under the
   * maximum delta config parameter.
   */
  m_angle = m_bias_generator->operator()(m_angle, rng);

  /*
   * Wandering is a combination of the current velocity, projected outward a
   * certain distance, and a displacement calculated at that point.
   *
   * There can be discontinuous jumps from a small positive angle for the wander
   * force to a large negative angle between timesteps and vice versa which
   * wreak havoc with controller's ability to compensate.
   *
   * So, compute the angle indirectly using sine and cosine in order to account
   * for sign differences and ensure continuity.
   */
  double angle = std::atan2(displacement.y() - circle_center.y(),
                            displacement.x() - circle_center.x());
  double angle_diff = angle - circle_center.angle().v();
  angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

  if (std::fabs(angle_diff - m_last_angle.v()) > M_PI) {
    angle_diff -= std::copysign(2 * M_PI, angle_diff);
  }
  m_last_angle = rmath::radians(angle_diff);

  rmath::vector2d wander((circle_center + displacement).length(),
                         rmath::radians(angle_diff));
  return wander.normalize() * mc_config.max;
} /* operator()() */

} /* namespace cosm::apf2D::nav */
