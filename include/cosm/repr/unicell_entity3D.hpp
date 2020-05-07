/**
 * \file unicell_entity3D.hpp
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

#ifndef INCLUDE_COSM_REPR_UNICELL_ENTITY3D_HPP_
#define INCLUDE_COSM_REPR_UNICELL_ENTITY3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <type_traits>

#include "rcppsw/math/range.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/entity3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class unicell_entity3D
 * \ingroup cosm repr
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
class unicell_entity3D : public entity3D {
 public:
  ~unicell_entity3D(void) override = default;

  /**
   * \brief Get the real location (center) of the object.
   */
  rmath::vector3d rpos3D(void) const override final { return m_rpos; }
  rmath::vector2d rpos2D(void) const override final {
    return m_rpos.project_on_xy();
  }

  /**
   * \brief Get the discretized coordinates of the center of the object.
   */
  rmath::vector3z dpos3D(void) const override final { return m_dpos; }
  rmath::vector2z dpos2D(void) const override final {
    return m_dpos.project_on_xy();
  }

  rmath::ranged xspan(void) const override final {
    return entity3D::xspan(rpos2D(), xdimr());
  }
  rmath::ranged yspan(void) const override final {
    return entity3D::yspan(rpos2D(), ydimr());
  }
  rmath::ranged zspan(void) const override final {
    return entity3D::zspan(rpos3D(), zdimr());
  }
  double xdimr(void) const override final { return m_dim.x(); }
  double ydimr(void) const override final { return m_dim.y(); }
  double zdimr(void) const override final { return m_dim.z(); }

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
  bool contains_point2D(const rmath::vector2d& point) const {
    return xspan().contains(point.x()) && yspan().contains(point.y());
  }
  bool contains_point3D(const rmath::vector3d& point) const {
    return xspan().contains(point.x()) && yspan().contains(point.y()) &&
           zspan().contains(point.z());
  }

  const rmath::vector3d& dims3D(void) const { return m_dim; }
  rmath::vector2d dims2D(void) const { return m_dim.project_on_xy(); }

 protected:
  unicell_entity3D(const rmath::vector3d& dim,
                   const rmath::vector3d& pos,
                   const rtypes::discretize_ratio& resolution)
      : unicell_entity3D{dim, pos, resolution, rtypes::constants::kNoUUID} {}

  unicell_entity3D(const rmath::vector3d& dim,
                   const rmath::vector3d& pos,
                   const rtypes::discretize_ratio& resolution,
                   const rtypes::type_uuid& id)
      : entity3D(id),
        m_dim(dim),
        m_rpos(pos),
        m_dpos(rmath::dvec2zvec(pos, resolution.v())) {}

  explicit unicell_entity3D(const rmath::vector3d& dim)
      : unicell_entity3D{dim, rtypes::constants::kNoUUID} {}

  unicell_entity3D(const rmath::vector3d& dim, const rtypes::type_uuid& id)
      : entity3D(id), m_dim(dim) {}

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   */
  template <typename T, RCPPSW_SFINAE_FUNC(T::is_movable())>
  void rpos3D(const rmath::vector3d& pos) {
    m_rpos = pos;
  }

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   */
  template <typename T, RCPPSW_SFINAE_FUNC(T::is_movable())>
  void dpos3D(const rmath::vector3z& pos) {
    m_dpos = pos;
  }

 private:
  /* clang-format off */
  rmath::vector3d m_dim;
  rmath::vector3d m_rpos{};
  rmath::vector3z m_dpos{};
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_UNICELL_ENTITY3D_HPP_ */
