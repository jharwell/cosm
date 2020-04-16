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
#include "rcppsw/math/range.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/entity_base.hpp"

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
 * \brief A base class from which all entities which can be represented in 2D
 * derive.
 */
class entity2D : public entity_base {
 public:
  /**
   * \brief Calculate the span in X of an entity given its location and
   * dimension in X.
   *
   * \return The span in X of the entity.
   */
  static rmath::ranged xspan(const rmath::vector2d& loc, double xdim) {
    return rmath::ranged(loc.x() - 0.5 * xdim, loc.x() + 0.5 * xdim);
  }

  /**
   * \brief Calculate the span in Y of an entity given its location and
   * dimension in Y.
   *
   * \return The span in Y of the entity.
   */
  static rmath::ranged yspan(const rmath::vector2d& loc, double ydim) {
    return rmath::ranged(loc.y() - 0.5 * ydim, loc.y() + 0.5 * ydim);
  }

  entity2D(void) = default;
  explicit entity2D(const rtypes::type_uuid& id) : entity_base(id) {}

  entity2D(const entity2D&) = default;
  entity2D& operator=(const entity2D&) = default;

  virtual ~entity2D(void) = default;

  /**
   * \brief Calculate the span in X of a 2D entity given its location and
   * dimension in X (objects track their own location and dimension).
   *
   * \return The span in X of the entity.
   */
  virtual rmath::ranged xspan(void) const = 0;

  /**
   * \brief Calculate the span in Y of a 2D entity given its location and
   * dimension in Y.
   *
   * \return The span in Y of the entity.
   */
  virtual rmath::ranged yspan(void) const = 0;

  /**
   * \brief Get the size of the 2D entity in the X direction in real coordinates.
   */
  virtual double xdimr(void) const = 0;

  /**
   * \brief Get the size of the 2D entity in the Y direction in real coordinates.
   */
  virtual double ydimr(void) const = 0;

  virtual rmath::vector2d rloc2D(void) const = 0;
  virtual rmath::vector2z dloc2D(void) const = 0;

  entity_dimensionality dimensionality(void) const override final {
    return entity_dimensionality::ek2D;
  }
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_ENTITY2D_HPP_ */
