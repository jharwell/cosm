/**
 * \file base_force.cpp
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/base_force.hpp"


/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d base_force::make_vel_floor(rmath::vector2d vel) {
  if (vel.length() <= std::numeric_limits<double>::epsilon()) {
    vel = rmath::vector2d::X * 0.01;
  }
  return vel;
} /* make_vel_floor() */

} /* namespace cosm::apf2D */
