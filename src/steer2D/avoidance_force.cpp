/**
 * \file avoidance_force.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "cosm/steer2D/avoidance_force.hpp"

#include "cosm/steer2D/config/avoidance_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
avoidance_force::avoidance_force(const config::avoidance_force_config* config)
    : mc_max(config->max) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d
avoidance_force::operator()(const boid&, const rmath::vector2d& closest) const {
  if (closest.length() > 0) {
    rmath::vector2d avoidance = -closest;
    return avoidance.normalize() * mc_max;
  } else {
    return { 0, 0 }; /* no threatening obstacles = no avoidance */
  }
} /* operator()() */

NS_END(steer2D, cosm);
