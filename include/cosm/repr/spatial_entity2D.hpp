/**
 * \file spatial_entity2D.hpp
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

#ifndef INCLUDE_COSM_REPR_SPATIAL_ENTITY2D_HPP_
#define INCLUDE_COSM_REPR_SPATIAL_ENTITY2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/range.hpp"
#include "rcppsw/types/spatial_dist.hpp"

#include "cosm/repr/spatial_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class spatial_entity2D
 * \ingroup cosm repr
 *
 * \brief Entity class from which all entities that which occupy spatial space
 * in the arena inherit. Note that occupying spatial space does not imply robots
 * can physically interact with the entity. Basically defines the interface for
 * a 2D bounding box for entities.
 */
class spatial_entity2D : public spatial_entity {
 public:
  /**
   * \brief Calculate the span in X in real coordinates of an entity given the
   * position of its anchor (2D or 3D) and dimension in X.
   *
   * \return The span in X of the entity.
   */
  template <typename TCoord>
  static rmath::ranged xrspan(const TCoord& anchor,
                              const rtypes::spatial_dist& xdim) {
    return { anchor.x(), (anchor.x() + xdim).v() };
  }

  /**
   * \brief Calculate the span in Y in real coordinates of an entity given the
   * position of its anchor (2D or 3D) and dimension in Y.
   *
   * \return The span in Y of the entity.
   */
  template <typename TCoord>
  static rmath::ranged yrspan(const TCoord& anchor,
                              const rtypes::spatial_dist& ydim) {
    return { anchor.y(), (anchor.y() + ydim).v() };
  }

  /**
   * \brief Calculate the span in X in discrete coordinates of an entity given
   * its discrete anchor (2D or 3D) and X dimension.
   *
   * This function can only be called for entities which have a defined discrete
   * center.
   *
   * \return The span in X of the entity (closed interval).
   */
  template <typename TCoord>
  static rmath::rangez xdspan(const TCoord& anchor, size_t xdim) {
    /* rely on truncation of the 0.5 remainder to 0 */
    return { anchor.x(), anchor.x() + xdim - 1 };
  }

  /**
   * \brief Calculate the span in Y in discrete coordinates of an entity given
   * its discrete cahor (2D or 3D) and Y dimension.
   *
   * This function can only be called for entities which have a defined discrete
   * center.
   *
   * \return The span in Y of the entity (closed interval).
   */
  template <typename TCoord>
  static rmath::rangez ydspan(const TCoord& anchor, size_t ydim) {
    /* rely on truncation of the 0.5 remainder to 0 */
    return { anchor.y(), anchor.y() + ydim - 1 };
  }

  spatial_entity2D(void) : spatial_entity2D{ rtypes::constants::kNoUUID } {}
  explicit spatial_entity2D(const rtypes::type_uuid& id) : spatial_entity(id) {}

  ~spatial_entity2D(void) override = default;

  /**
   * \brief Calculate the span in X of th entity in real coordinates.
   */
  virtual rmath::ranged xrspan(void) const = 0;

  /**
   * \brief Calculate the span in Y of the entity in real coordinates.
   */
  virtual rmath::ranged yrspan(void) const = 0;

  /**
   * \brief Calculate the span in X of the entity in discrete coordinates.
   *
   * \return The span in X of the entity.
   */
  virtual rmath::rangez xdspan(void) const = 0;

  /**
   * \brief Calculate the span in Y of the entity in discrete coordinates.
   */
  virtual rmath::rangez ydspan(void) const = 0;

  /**
   * \brief Get the size of the entity in the X direction in real
   * coordinates.
   */
  virtual rtypes::spatial_dist xrsize(void) const = 0;

  /**
   * \brief Get the size of the entity in the Y direction in real
   * coordinates.
   */
  virtual rtypes::spatial_dist yrsize(void) const = 0;

  /**
   * \brief Get the size of the entity in the X direction in discrete
   * coordinates.
   */
  virtual size_t xdsize(void) const = 0;

  /**
   * \brief Get the size of the entity in the Y direction in discrete
   * coordinates.
   */
  virtual size_t ydsize(void) const = 0;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_SPATIAL_ENTITY2D_HPP_ */
