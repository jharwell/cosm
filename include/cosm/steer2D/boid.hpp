/**
 * \file boid.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/kin/odometry.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class boid
 * \ingroup steer2D
 *
 * \brief Interface representing an entity upon which kinematic forces can act
 * (i.e. any class that wants to use the \ref force_calculator must conform to
 * this interface).
 */
class boid {
 public:
  boid(void) = default;
  virtual ~boid(void) = default;

  /**
   * \brief Should return the odometry of the entity.
   */
  virtual ckin::odometry odometry(void) const = 0;

  /**
   * \brief Should return the maximum linear speed of the entity. This can vary
   * in time, if desired.
   */
  virtual double max_linear_speed(void) const = 0;
};

NS_END(steer2D, cosm);
