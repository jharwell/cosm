/**
 * \file wander_force.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
nn *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/steer2D/wander_force.hpp"

#include "rcppsw/math/angles.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/steer2D/config/wander_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
wander_force::wander_force(const config::wander_force_config* const config)
    : mc_use_normal(config->normal_dist),
      mc_max(config->max),
      mc_circle_distance(config->circle_distance),
      mc_circle_radius(config->circle_radius),
      mc_max_angle_delta(config->max_angle_delta),
      m_interval(config->interval) {}

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
  if (m_count % m_interval != 0) {
    return {0, 0};
  }

  /* calculate circle center */
  rmath::vector2d velocity;
  if (entity.linear_velocity().length() <= 0) {
    velocity = rmath::vector2d(1, 0);
  } else {
    velocity = entity.linear_velocity();
  }

  rmath::vector2d circle_center =
      (velocity).normalize().scale(mc_circle_distance);

  /* calculate displacement force (the actual wandering) */
  rmath::vector2d displacement(
      mc_circle_radius * std::cos((m_angle + velocity.angle()).v()),
      mc_circle_radius * std::sin((m_angle + velocity.angle()).v()));

  /*
   * Update wander angle so it won't have the same value next time with a
   * random pertubation in the range [-max delta, max_delta].
   */
  double val;
  if (mc_use_normal) {
    /*
     * Both min and max are 3 std deviations away from the mean of 0, so it is
     * very unlikely that we will get a value outside the max deviation. If we
     * do, just shrink the max angle in the input parameters.
     */
    val = rng->gaussian(0, 2 * mc_max_angle_delta / 6.0);
  } else {
    val = -mc_max_angle_delta + 2 * mc_max_angle_delta * rng->uniform(0.0, 1.0);
  }
  rmath::degrees perturbation(
      std::fmod(rmath::to_degrees(m_angle).v() + val, mc_max_angle_delta));
  m_angle = rmath::to_radians(perturbation);

  /*
   * Wandering is a combination of the current velocity, projected outward a
   * certain distance, and a displacement calculated at that point.
   *
   * There can be discontinuous jumps from a small positive angle for the wander
   * force to a large negative angle between timesteps and vice versa which play
   * havoc with controller' ability to compensate.
   *
   * So, compute the angle indirectly using sine and cosine in order to account
   * for sign differences and ensure continuity.
   */
  double angle = std::atan2(displacement.y() - circle_center.y(),
                            displacement.x() - circle_center.x());
  double angle_diff = angle - circle_center.angle().v();
  angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

  if (std::fabs(angle_diff - m_last_angle) > M_PI) {
    angle_diff -= std::copysign(2 * M_PI, angle_diff);
  }
  m_last_angle = angle_diff;

  rmath::vector2d wander((circle_center + displacement).length(),
                         rmath::radians(angle_diff));
  return wander.normalize() * mc_max;
} /* operator()() */

NS_END(steer2D, cosm);
