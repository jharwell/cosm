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
  rmath::vector2d rpos2D(void) const override final { return m_rpos; }

  /**
   * \brief Get the discretized coordinates of the center of the object.
   */
  rmath::vector2z dpos2D(void) const override final { return m_dpos; }

  rmath::ranged xspan(void) const override final {
    return entity2D::xspan(rpos2D(), xdimr());
  }
  rmath::ranged yspan(void) const override final {
    return entity2D::yspan(rpos2D(), ydimr());
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
  bool contains_point2D(const rmath::vector2d& point) const {
    return xspan().contains(point.x()) && yspan().contains(point.y());
  }

  const rmath::vector2d& dims2D(void) const { return m_dim; }

 protected:
  unicell_entity2D(const rmath::vector2d& dim,
                   const rmath::vector2d& pos,
                   const rtypes::discretize_ratio& resolution)
      : unicell_entity2D{dim, pos, resolution, rtypes::constants::kNoUUID} {}

  unicell_entity2D(const rmath::vector2d& dim,
                   const rmath::vector2d& pos,
                   const rtypes::discretize_ratio& resolution,
                   const rtypes::type_uuid& id)
      : entity2D(id),
        m_dim(dim),
        m_rpos(pos),
        m_dpos(rmath::dvec2zvec(pos, resolution.v())) {}

  explicit unicell_entity2D(const rmath::vector2d& dim)
      : unicell_entity2D{dim, rtypes::constants::kNoUUID} {}

  unicell_entity2D(const rmath::vector2d& dim, const rtypes::type_uuid& id)
      : entity2D(id), m_dim(dim) {}

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   */
  template <typename T, RCPPSW_SFINAE_FUNC(T::is_movable())>
  void rpos2D(const rmath::vector2d& pos) {
    m_rpos = pos;
  }

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   */
  template <typename T, RCPPSW_SFINAE_FUNC(T::is_movable())>
  void dpos2D(const rmath::vector2z& pos) {
    m_dpos = pos;
  }

 private:
  /* clang-format off */
  rmath::vector2d m_dim;
  rmath::vector2d m_rpos{};
  rmath::vector2z m_dpos{};
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_UNICELL_ENTITY2D_HPP_ */
