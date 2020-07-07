/**
 * \file spatial_entity.hpp
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

#ifndef INCLUDE_COSM_REPR_SPATIAL_ENTITY_HPP_
#define INCLUDE_COSM_REPR_SPATIAL_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/math/range.hpp"
#include "rcppsw/types/spatial_dist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/entity_dimensionality.hpp"
#include "cosm/repr/base_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class spatial_entity
 * \ingroup cosm repr
 *
 * \brief Entity class from which all entities that which occupy spatial space
 * in the arena. Note that occupying spatial space does not imply robots can
 * physically interact with the entity.
 */
class spatial_entity : public base_entity {
 public:
    /**
   * \brief Calculate the span in X in real coordinates of an entity given the
   * position of its anchor (2D or 3D) and dimension in X.
   *
   * \return The span in X of the entity.
   */
  template<typename TCoord>
  static rmath::ranged xrspan(const TCoord& anchor,
                              const rtypes::spatial_dist& xdim) {
    return {anchor.x(), (anchor.x() + xdim).v()};
  }

  /**
   * \brief Calculate the span in Y in real coordinates of an entity given the
   * position of its anchor (2D or 3D) and dimension in Y.
   *
   * \return The span in Y of the entity.
   */
  template<typename TCoord>
  static rmath::ranged yrspan(const TCoord& anchor,
                              const rtypes::spatial_dist& ydim) {
    return {anchor.y(), (anchor.y() + ydim).v()};
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
  template<typename TCoord>
  static rmath::rangez xdspan(const TCoord& anchor, size_t xdim) {
    /* rely on truncation of the 0.5 remainder to 0 */
    return {anchor.x(), anchor.x() + xdim - 1};
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
  template<typename TCoord>
  static rmath::rangez ydspan(const TCoord& anchor, size_t ydim) {
    /* rely on truncation of the 0.5 remainder to 0 */
    return {anchor.y(), anchor.y() + ydim - 1};
  }

  spatial_entity(void) : spatial_entity{rtypes::constants::kNoUUID} {}
  explicit spatial_entity(const rtypes::type_uuid& id) : base_entity(id) {}

  spatial_entity(const spatial_entity&) = default;
  spatial_entity& operator=(const spatial_entity&) = default;

  ~spatial_entity(void) override = default;

  /**
   * \brief Return whether the entity is 2D or 3D.
   */
  virtual entity_dimensionality dimensionality(void) const = 0;

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

#endif /* INCLUDE_COSM_REPR_SPATIAL_ENTITY_HPP_ */
