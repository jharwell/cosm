/**
 * \file avoidance_force.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
