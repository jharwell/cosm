/**
 * \file unicell_entity2D.hpp
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
#include <type_traits>

#include "rcppsw/math/range.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class unicell_entity2D
 * \ingroup repr
 *
 * \brief Representation of a 2D entity in the arena in two dimensions.
 *
 * - Resides in a single cell ("unicell")
 *
 * - Has an extent that can span multiple cells, even though the entity itself
 *   is confined to a single cell. A useful, simplifying abstraction in dealing
 *   with the kinds of entities that robots interact with (blocks, caches,
 *   etc.), because the handshaking logic is much simpler.
 */
class unicell_entity2D : public entity2D, public rer::client<unicell_entity2D> {
 public:
  using entity2D::danchor2D;
  using entity2D::ranchor2D;

  ~unicell_entity2D(void) override = default;

  rmath::vector2z dcenter2D(void) const {
    ER_ASSERT(RCPPSW_IS_ODD(dbb().xsize()) && RCPPSW_IS_ODD(dbb().ysize()),
              "%s called on entity without defined center",
              __FUNCTION__);
    return entity2D::dcenter2D();
  }

  /**
   * \brief Return if a real-valued point lies within the extent of the 2D
   * entity for:
   *
   * 1. Visualization purposes.
   * 2. Determining if a robot is on top of an entity.
   *
   * \param point The point to check.
   *
   * \return \c TRUE if the condition is met, and \c FALSE otherwise.
   */
  bool contains_point(const rmath::vector2d& point) const {
    return xrspan().contains(point.x()) && yrspan().contains(point.y());
  }

  /**
   * \brief Return if a discrete cell lies within the extent of the 2D entity.
   *
   * \param cell The point to check.
   *
   * \return \c TRUE if the condition is met, and \c FALSE otherwise.
   */
  bool contains_cell(const rmath::vector2z& cell) const {
    return xdspan().contains(cell.x()) && ydspan().contains(cell.y());
  }

 protected:
  unicell_entity2D(const rtypes::type_uuid& id,
                   const rmath::vector2d& dims,
                   const rtypes::discretize_ratio& resolution)
      : entity2D(id, rmath::vector3d(dims, 0.0), rtypes::spatial_dist(resolution.v())),
        ER_CLIENT_INIT("cosm.repr.unicell_entity2D"),
        mc_arena_res(resolution) {}

  unicell_entity2D(const rtypes::type_uuid& id,
                   const rmath::vector2d& dims,
                   const rmath::vector2d& center,
                   const rtypes::discretize_ratio& resolution)
      : entity2D(id,
                 rmath::vector3d(dims, 0.0),
                 rmath::vector3d(center, 0.0),
                 rtypes::spatial_dist(resolution.v())),
        ER_CLIENT_INIT("cosm.repr.unicell_entity2D"),
        mc_arena_res(resolution) {}

  const rtypes::discretize_ratio& arena_res(void) const { return mc_arena_res; }

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   *
   * Updates the real anchor,center. Does not change the discrete anchor,center
   * of the entity.
   */
  template <typename T, RCPPSW_SFINAE_DECLDEF(T::is_movable())>
  void ranchor2D(const rmath::vector2d& ranchor) {
    rbb().update(rmath::vector3d(ranchor, 0.0));
  }

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   *
   * Updates the discrete anchor,center. Does not change the real anchor,center
   * of the entity.
   */
  template <typename T, RCPPSW_SFINAE_DECLDEF(T::is_movable())>
  void danchor2D(const rmath::vector2z& danchor) {
    dbb().update(rmath::vector3z(danchor, 0));
  }

 private:
  /* clang-format off */
  const rtypes::discretize_ratio mc_arena_res;
  /* clang-format on */
};

NS_END(repr, cosm);

