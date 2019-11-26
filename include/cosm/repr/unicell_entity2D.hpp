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

#ifndef INCLUDE_COSM_REPR_UNICELL_ENTITY2D_HPP_
#define INCLUDE_COSM_REPR_UNICELL_ENTITY2D_HPP_

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
 * \ingroup cosm repr
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
class unicell_entity2D : public entity2D {
 public:
  ~unicell_entity2D(void) override = default;

  /**
   * \brief Get the real location (center) of the object.
   */
  const rmath::vector2d& rloc(void) const { return m_rloc; }

  /**
   * \brief Get the discretized coordinates of the center of the object.
   */
  const rmath::vector2u& dloc(void) const { return m_dloc; }

  rmath::ranged xspan(void) const override final {
    return entity2D::xspan(rloc(), xdimr());
  }
  rmath::ranged yspan(void) const override final {
    return entity2D::yspan(rloc(), ydimr());
  }
  double xdimr(void) const override final { return m_dim.x(); }
  double ydimr(void) const override final { return m_dim.y(); }

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
    return xspan().contains(point.x()) && yspan().contains(point.y());
  }

  const rmath::vector2d& dims(void) const { return m_dim; }

 protected:
  unicell_entity2D(const rmath::vector2d& dim,
                   const rmath::vector2d& loc,
                   rtypes::discretize_ratio resolution)
      : unicell_entity2D{dim, loc, resolution, -1} {}

  unicell_entity2D(const rmath::vector2d& dim,
                   const rmath::vector2d& loc,
                   rtypes::discretize_ratio resolution,
                   int id)
      : entity2D(id),
        m_dim(dim),
        m_rloc(loc),
        m_dloc(rmath::dvec2uvec(loc, resolution.v())) {}

  explicit unicell_entity2D(const rmath::vector2d& dim)
      : unicell_entity2D{dim, -1} {}

  unicell_entity2D(const rmath::vector2d& dim, int id)
      : entity2D(id), m_dim(dim), m_rloc(), m_dloc() {}

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   */
  template <typename T, RCPPSW_SFINAE_FUNC(T::is_movable())>
  void rloc(const rmath::vector2d& loc) {
    m_rloc = loc;
  }

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   */
  template <typename T, RCPPSW_SFINAE_FUNC(T::is_movable())>
  void dloc(const rmath::vector2u& loc) {
    m_dloc = loc;
  }

 private:
  /* clang-format off */
  rmath::vector2d m_dim;
  rmath::vector2d m_rloc;
  rmath::vector2u m_dloc;
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_UNICELL_ENTITY2D_HPP_ */
