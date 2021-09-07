/**
 * \file spatial_entity3D.hpp
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

#ifndef INCLUDE_COSM_REPR_SPATIAL_ENTITY3D_HPP_
#define INCLUDE_COSM_REPR_SPATIAL_ENTITY3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"

#include "cosm/repr/spatial_entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class spatial_entity3D
 * \ingroup cosm repr
 *
 * \brief Base class from which all arena spatial entities which can be
 * represented in 3D derive. Basically defines the interface for a 2D bounding
 * box for entities.
 */
class spatial_entity3D : public spatial_entity2D {
 public:
  using spatial_entity2D::spatial_entity2D;

  /**
   * \brief Calculate the span in Z in real coordinates of an entity given the
   * position of its 3D center and dimension in Z.
   *
   * \return The span in Z of the entity.
   */
  static rmath::ranged zrspan(const rmath::vector3d& anchor,
                              const rtypes::spatial_dist& zdim) {
    return { anchor.z(), (anchor.z() + zdim).v() };
  }

  /**
   * \brief Calculate the span in Z in discrete coordinates of an entity given
   * its discrete anchor and Z dimension.
   *
   * This function can only be called for entities which have a defined discrete
   * anchor.
   *
   * \return The span in Z of the entity (closed interval).
   */
  static rmath::rangez zdspan(const rmath::vector3z& anchor, size_t zdim) {
    /* rely on truncation of the 0.5 remainder to 0 */
    return { anchor.z(), anchor.z() + zdim - 1 };
  }

  ~spatial_entity3D(void) override = default;

  /**
   * \brief Calculate the span in Z of a 3D entity given its location and
   * dimension in Z.
   *
   * \return The span in Z of the entity.
   */
  virtual rmath::ranged zrspan(void) const = 0;

  /**
   * \brief Get the size of the 3D entity in the Z direction in real
   * coordinates.
   */
  virtual rtypes::spatial_dist zrsize(void) const = 0;

  /**
   * \brief Calculate the span in Z of the entity in discrete coordinates.
   */
  virtual rmath::rangez zdspan(void) const = 0;

  /**
   * \brief Get the size of the 3D entity in the Z direction in discrete
   * coordinates.
   */
  virtual size_t zdsize(void) const = 0;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_SPATIAL_ENTITY3D_HPP_ */
