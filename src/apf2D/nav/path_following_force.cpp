/**
 * \file path_following_force.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/path_following_force.hpp"

#include "cosm/apf2D/nav/config/path_following_force_config.hpp"
#include "cosm/nav/trajectory.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
path_following_force::path_following_force(
    const config::path_following_force_config* config)
    : mc_max(config->max),
      mc_radius(config->radius),
      m_seek(mc_max) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d
path_following_force::operator()(const boid& entity,
                                 cnav::trajectory* state) const {
  auto next_point = state->next_point();
  auto odom = entity.odometry();
  if ((odom.pose.position.to_2D() - next_point.to_2D()).length() <= mc_radius) {
    state->mark_progress(1);
  }
  if (state->is_complete()) {
    return { 0.0, 0.0 }; /* reached the end of the path */
  } else {
    return m_seek(entity, state->next_point().to_2D());
  }
} /* operator()() */

} /* namespace cosm::apf2D::nav */
