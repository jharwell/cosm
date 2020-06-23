/**
 * \file entity2D.hpp
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

#ifndef INCLUDE_COSM_REPR_ENTITY2D_HPP_
#define INCLUDE_COSM_REPR_ENTITY2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/spatial_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class entity2D
 * \ingroup cosm repr
 *
 * \brief A base class from which all spatial entities which can be represented
 * in 2D derive.
 */
class entity2D : public spatial_entity {
 public:
  using spatial_entity::spatial_entity;

  entity2D(const entity2D&) = default;
  entity2D& operator=(const entity2D&) = default;

  virtual ~entity2D(void) = default;

  /**
   * \brief Return the anchor (LL corner) of the object in real coordinates.
   */
  virtual rmath::vector2d ranchor2D(void) const = 0;

  /**
   * \brief Return the 2D center of the object in real coordinates. This ALWAYS
   * exists, even if the \ref dcenter2D() does not.
   */
  virtual rmath::vector2d rcenter2D(void) const = 0;

  /**
   * \brief Return the anchor (LL corner) of the object in discrete
   * coordinates. This ALWAYS exists, evenif if \ref center2D() does not.
   */
  virtual rmath::vector2z danchor2D(void) const = 0;

  /*
   * \brief Return the center of the object in discrete coordinates,
   * \a IF it exists. If the entity X,Y dimensions are both odd, then it exists,
   * otherwise, it does not.
   *
   * If this function is called on an entity which has no center, an assertion
   * should be triggered.
   */
  virtual rmath::vector2z dcenter2D(void) const = 0;

  entity_dimensionality dimensionality(void) const override final {
    return entity_dimensionality::ek2D;
  }
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_ENTITY2D_HPP_ */
