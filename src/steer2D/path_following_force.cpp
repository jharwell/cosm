/**
 * \file path_following_force.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
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
#include "cosm/steer2D/path_following_force.hpp"

#include "cosm/steer2D/config/path_following_force_config.hpp"
#include "cosm/steer2D/ds/path_state.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
path_following_force::path_following_force(
    const config::path_following_force_config* config)
    : mc_max(config->max), mc_radius(config->radius), m_seek(mc_max) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d
path_following_force::operator()(const boid& entity,
                                 csteer2D::ds::path_state* state) const {
  auto next_point = state->next_point();
  if ((entity.pos2D() - next_point).length() <= mc_radius) {
    state->mark_progress(1);
  }
  if (state->is_complete()) {
    return { 0.0, 0.0 }; /* reached the end of the path */
  } else {
    return m_seek(entity, state->next_point());
  }
} /* operator()() */

NS_END(steer2D, cosm);
