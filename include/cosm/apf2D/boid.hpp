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
namespace cosm::apf2D {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class boid
 * \ingroup apf2D
 *
 * \brief Interface representing an entity upon which kinematic forces can act
 * (i.e. any class that wants to use the \ref capf2D::apf_manager must conform
 * to this interface).
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
   * \brief Should return the magnitude of the maximum velocity of the
   * entity. This can vary in time, if desired.
   */
  virtual double max_velocity(void) const = 0;
};

} /* namespace cosm::apf2D */
