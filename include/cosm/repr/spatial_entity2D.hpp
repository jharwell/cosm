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

#pragma once

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
 * \ingroup repr
 *
 * \brief Entity class from which all entities that which occupy spatial space
 * in the arena inherit. Note that occupying spatial space does not imply robots
 * can physically interact with the entity. Basically defines the interface for
 * a 2D bounding box for entities.
 */
class spatial_entity2D : public spatial_entity {
 public:
  using spatial_entity::spatial_entity;

  ~spatial_entity2D(void) override = default;

  /**
   * \brief Calculate the span in X of th entity in real coordinates.
   */
  rmath::ranged xrspan(void) const { return rbb().xspan(); }

  /**
   * \brief Calculate the span in Y of the entity in real coordinates.
   */
  rmath::ranged yrspan(void) const { return rbb().yspan(); }

  /**
   * \brief Get the size of the entity in the X direction in real
   * coordinates.
   */
  rtypes::spatial_dist xrsize(void) const {
    return rtypes::spatial_dist(rbb().xsize());
  }

  /**
   * \brief Get the size of the entity in the Y direction in real
   * coordinates.
   */
  rtypes::spatial_dist yrsize(void) const {
    return rtypes::spatial_dist(rbb().ysize());
  }

  /**
   * \brief Calculate the span in X of the entity in discrete coordinates.
   *
   * \return The span in X of the entity.
   */
  rmath::rangez xdspan(void) const { return dbb().xspan(); }

  /**
   * \brief Calculate the span in Y of the entity in discrete coordinates.
   */
  rmath::rangez ydspan(void) const { return dbb().yspan(); }

  /**
   * \brief Get the size of the entity in the X direction in discrete
   * coordinates.
   */
  size_t xdsize(void) const { return dbb().xsize(); }

  /**
   * \brief Get the size of the entity in the Y direction in discrete
   * coordinates.
   */
  size_t ydsize(void) const { return dbb().ysize(); }
};

NS_END(repr, cosm);
