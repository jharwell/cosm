/**
 * \file unicell_entity3D.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <type_traits>

#include "rcppsw/math/range.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/entity3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class unicell_entity3D
 * \ingroup repr
 *
 * \brief Representation of a 3D entity in the arena.
 *
 * - Resides in a single cell ("unicell")
 *
 * - Has an extent that can span multiple cells, even though the entity itself
 *   "resides" in a single cell. A useful, simplifying abstraction in dealing
 *   with the kinds of entities that robots interact with because the
 *   handshaking logic is much simpler.
 */
class unicell_entity3D : public entity3D, public rer::client<unicell_entity3D> {
 public:
  using entity3D::danchor3D;
  using entity3D::ranchor3D;

  ~unicell_entity3D(void) override = default;

  rmath::vector3z dcenter3D(void) const {
    ER_ASSERT(RCPPSW_IS_ODD(dbb().xsize()) && RCPPSW_IS_ODD(dbb().ysize()) &&
                  RCPPSW_IS_ODD(dbb().zsize()),
              "dcenter3D() called on entity without defined center");
    return entity3D::dcenter3D();
  }

  /**
   * \brief Return if a real-valued point lies within the extent of the 3D
   * entity in 2D for:
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
  bool contains_point(const rmath::vector3d& point) const {
    return xrspan().contains(point.x()) && yrspan().contains(point.y()) &&
           zrspan().contains(point.z());
  }

 protected:
  unicell_entity3D(const rtypes::type_uuid& id,
                   const rmath::vector3d& dims,
                   const rtypes::discretize_ratio& resolution)
      : entity3D(id, dims, rspatial::euclidean_dist(resolution.v())),
        ER_CLIENT_INIT("cosm.repr.unicell_entity3D"),
        mc_arena_res(resolution) {}

  unicell_entity3D(const rtypes::type_uuid& id,
                   const rmath::vector3d& dims,
                   const rmath::vector3d& center,
                   const rtypes::discretize_ratio& resolution)
      : entity3D(id, dims, center, rspatial::euclidean_dist(resolution.v())),
        ER_CLIENT_INIT("cosm.repr.unicell_entity3D"),
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
  void ranchor3D(const rmath::vector3d& ranchor) {
    rbb().update(ranchor);
  }

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   *
   * Updates the discrete anchor,center. Does not change the real anchor,center
   * of the entity.
   */
  template <typename T, RCPPSW_SFINAE_DECLDEF(T::is_movable())>
  void danchor3D(const rmath::vector3z& danchor) {
    dbb().update(danchor);
  }

 private:
  /* clang-format off */
  const rtypes::discretize_ratio mc_arena_res;
  /* clang-format on */
};

} /* namespace cosm::repr */
