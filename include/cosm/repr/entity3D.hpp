/**
 * \file entity3D.hpp
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

#ifndef INCLUDE_COSM_REPR_ENTITY3D_HPP_
#define INCLUDE_COSM_REPR_ENTITY3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"

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
 * \class entity3D
 * \ingroup cosm repr
 *
 * \brief Base class from which all arena entities which can be represented in
 * 3D derive.
 */
class entity3D : public entity_base {
 public:
  /**
   * \brief Calculate the span in X of an entity given its location and
   * dimension in X.
   *
   * \return The span in X of the entity.
   */
  static rmath::ranged xspan(const rmath::vector3d& loc, double xdim) {
    return rmath::ranged(loc.x() - 0.5 * xdim, loc.x() + 0.5 * xdim);
  }

  /**
   * \brief Calculate the span in Y of an entity given its location and
   * dimension in Y.
   *
   * \return The span in Y of the entity.
   */
  static rmath::ranged yspan(const rmath::vector3d& loc, double ydim) {
    return rmath::ranged(loc.y() - 0.5 * ydim, loc.y() + 0.5 * ydim);
  }
  /**
   * \brief Calculate the span in Z of an entity given its location and
   * dimension in Z.
   *
   * \return The span in Z of the entity.
   */
  static rmath::ranged zspan(const rmath::vector3d& loc, double zdim) {
    return rmath::ranged(loc.z() - 0.5 * zdim, loc.z() + 0.5 * zdim);
  }

  entity3D(void) = default;
  explicit entity3D(const rtypes::type_uuid& id) : entity_base(id) {}

  entity3D(const entity3D&) = default;
  entity3D& operator=(const entity3D&) = default;

  virtual ~entity3D(void) = default;

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
   * \brief Calculate the span in Z of a 3D entity given its location and
   * dimension in Z.
   *
   * \return The span in Z of the entity.
   */
  virtual rmath::ranged zspan(void) const = 0;

  /**
   * \brief Get the size of the 2D entity in the X direction in real coordinates.
   */
  virtual double xdimr(void) const = 0;

  /**
   * \brief Get the size of the 2D entity in the Y direction in real coordinates.
   */
  virtual double ydimr(void) const = 0;

  /**
   * \brief Get the size of the 3D entity in the Z direction in real coordinates.
   */
  virtual double zdimr(void) const = 0;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_ENTITY3D_HPP_ */
