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
class unicell_entity2D : public entity2D,
                         public rer::client<unicell_entity2D> {
 public:
  ~unicell_entity2D(void) override = default;

  rmath::vector2d rcenter2D(void) const override final {
    return m_rcenter;
  }
  rmath::vector2d ranchor2D(void) const override final {
    return m_ranchor;
  }
  rmath::ranged xrspan(void) const override final {
    return spatial_entity::xrspan(ranchor2D(), xrsize());
  }
  rmath::ranged yrspan(void) const override final {
    return spatial_entity::yrspan(ranchor2D(), yrsize());
  }
  rtypes::spatial_dist xrsize(void) const override final {
    return rtypes::spatial_dist(m_rdim.x()); }
  rtypes::spatial_dist yrsize(void) const override final {
    return rtypes::spatial_dist(m_rdim.y());
  }

  RCSW_PURE rmath::vector2z dcenter2D(void) const override final {
    ER_ASSERT(RCSW_IS_ODD(m_ddim.x()) && RCSW_IS_ODD(m_ddim.y()),
              "dcenter2D() called on entity%d without defined center: dims=%s",
              id().v(),
              rcppsw::to_string(m_ddim).c_str());
    return m_dcenter;
  }
  rmath::vector2z danchor2D(void) const override final {
    return m_danchor;
  }
  rmath::rangez xdspan(void) const override final {
    return spatial_entity::xdspan(danchor2D(), xdsize());
  }
  rmath::rangez ydspan(void) const override final {
    return spatial_entity::ydspan(danchor2D(), ydsize());
  }
  size_t xdsize(void) const override final { return m_ddim.x(); }
  size_t ydsize(void) const override final { return m_ddim.y(); }

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
    return xrspan().contains(point.x()) && yrspan().contains(point.y());
  }

  /**
   * \brief Return if a discrete cell lies within the extent of the 2D entity.
   *
   * \param cell The point to check.
   *
   * \return \c TRUE if the condition is met, and \c FALSE otherwise.
   */
  bool contains_cell2D(const rmath::vector2z& cell) const {
    return xdspan().contains(cell.x()) && ydspan().contains(cell.y());
  }

  const rmath::vector2d& rdim2D(void) const { return m_rdim; }
  const rmath::vector2z& ddim2D(void) const { return m_ddim; }

 protected:
  unicell_entity2D(const rtypes::type_uuid& id,
                   const rmath::vector2d& rdim,
                   const rtypes::discretize_ratio& resolution)
      : unicell_entity2D{id, rdim, resolution, rmath::vector2d()} {}

  unicell_entity2D(const rtypes::type_uuid& id,
                   const rmath::vector2d& rdim,
                   const rtypes::discretize_ratio& resolution,
                   const rmath::vector2d& rcenter);

  const rtypes::discretize_ratio& arena_res(void) const { return mc_arena_res; }

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   *
   * Updates the real anchor,center. Does not change the discrete anchor,center
   * of the entity.
   */
  template <typename T, RCPPSW_SFINAE_FUNC(T::is_movable())>
  void ranchor2D(const rmath::vector2d& ranchor) {
    m_ranchor = ranchor;
    m_rcenter = m_ranchor + m_rdim / 2.0;
  }

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   *
   * Updates the discrete anchor,center. Does not change the real anchor,center
   * of the entity.
   */
  template <typename T, RCPPSW_SFINAE_FUNC(T::is_movable())>
  void danchor2D(const rmath::vector2z& danchor) {
    m_danchor = danchor;
    m_dcenter = m_danchor + m_ddim / 2;
  }

 private:
  /* clang-format off */
  const rtypes::discretize_ratio mc_arena_res;

  rmath::vector2d                m_rdim;
  rmath::vector2d                m_rcenter{};
  rmath::vector2d                m_ranchor{};

  rmath::vector2z                m_ddim;
  rmath::vector2z                m_dcenter{};
  rmath::vector2z                m_danchor{};
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_UNICELL_ENTITY2D_HPP_ */
