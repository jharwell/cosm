/**
 * \file boid.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
