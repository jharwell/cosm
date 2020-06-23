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
#include "rcppsw/types/spatial_dist.hpp"

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
class unicell_entity3D : public entity3D,
                         public rer::client<unicell_entity3D> {
 public:
  ~unicell_entity3D(void) override = default;

  rmath::vector3d rcenter3D(void) const override final {
    return m_rcenter;
  }
  rmath::vector3d ranchor3D(void) const override final {
    return m_ranchor;
  }
  rmath::vector2d rcenter2D(void) const override final {
    return rcenter3D().to_2D();
  }
  rmath::vector2d ranchor2D(void) const override final {
    return ranchor3D().to_2D();
  }
  rmath::ranged xrspan(void) const override final {
    return spatial_entity::xrspan(ranchor3D(), xrsize());
  }
  rmath::ranged yrspan(void) const override final {
    return spatial_entity::yrspan(ranchor3D(), yrsize());
  }
  rmath::ranged zrspan(void) const override final {
    return entity3D::zrspan(ranchor3D(), yrsize());
  }
  rtypes::spatial_dist xrsize(void) const override final {
    return rtypes::spatial_dist(m_rdim.x()); }
  rtypes::spatial_dist yrsize(void) const override final {
    return rtypes::spatial_dist(m_rdim.y());
  }
  rtypes::spatial_dist zrsize(void) const override final {
    return rtypes::spatial_dist(m_rdim.z());
  }

  rmath::vector3z dcenter3D(void) const override final {
    ER_ASSERT(RCSW_IS_ODD(m_ddim.x()) &&
              RCSW_IS_ODD(m_ddim.y()) &&
              RCSW_IS_ODD(m_ddim.z()),
              "dcenter3D() called on entity without defined center");
    return m_dcenter;
  }

  rmath::vector2z dcenter2D(void) const override final {
    return dcenter3D().to_2D();
  }
  rmath::vector3z danchor3D(void) const override final {
    return m_danchor;
  }
  rmath::vector2z danchor2D(void) const override final {
    return danchor3D().to_2D();
  }
  rmath::rangez xdspan(void) const override final {
    return spatial_entity::xdspan(danchor2D(), xdsize());
  }
  rmath::rangez ydspan(void) const override final {
    return spatial_entity::ydspan(danchor2D(), ydsize());
  }
  rmath::rangez zdspan(void) const override final {
    return entity3D::ydspan(danchor2D(), zdsize());
  }
  size_t xdsize(void) const override final { return m_ddim.x(); }
  size_t ydsize(void) const override final { return m_ddim.y(); }
  size_t zdsize(void) const override final { return m_ddim.z(); }

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
    return xrspan().contains(point.x()) && yrspan().contains(point.y());
  }
  bool contains_point3D(const rmath::vector3d& point) const {
    return xrspan().contains(point.x()) && yrspan().contains(point.y()) &&
           zrspan().contains(point.z());
  }

  const rmath::vector3d& rdim3D(void) const { return m_rdim; }
  rmath::vector2d rdim2D(void) const { return rdim3D().to_2D(); }

 protected:
  unicell_entity3D(const rtypes::type_uuid& id,
                   const rmath::vector3d& rdim,
                   const rtypes::discretize_ratio& resolution)
      : unicell_entity3D{id, rdim, resolution, rmath::vector3d()} {}

  unicell_entity3D(const rtypes::type_uuid& id,
                   const rmath::vector3d& rdim,
                   const rtypes::discretize_ratio& resolution,
                   const rmath::vector3d& rcenter)
      : entity3D(id),
        ER_CLIENT_INIT("cosm.repr.unicell_entity3D"),
        mc_arena_res(resolution),
        m_rdim(rdim),
        m_rcenter(rcenter),
        m_ranchor(m_rcenter - m_rdim / 2.0),
        m_ddim(rmath::dvec2zvec(m_rdim, mc_arena_res.v())),
        m_dcenter(rmath::dvec2zvec(m_rcenter, mc_arena_res.v())),
        m_danchor(m_dcenter - m_ddim / 2) {}

  const rtypes::discretize_ratio& arena_res(void) const { return mc_arena_res; }

  /**
   * \brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   *
   * Updates the real anchor,center. Does not change the discrete anchor,center
   * of the entity.
   */
  template <typename T, RCPPSW_SFINAE_FUNC(T::is_movable())>
  void ranchor3D(const rmath::vector3d& ranchor) {
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
  void danchor3D(const rmath::vector3z& danchor) {
    m_danchor = danchor;
    m_dcenter = m_danchor + m_ddim / 2.0;
  }

 private:
  /* clang-format off */
  const rtypes::discretize_ratio mc_arena_res;

  rmath::vector3d                m_rdim;
  rmath::vector3d                m_rcenter{};
  rmath::vector3d                m_ranchor{};

  rmath::vector3z                m_ddim;
  rmath::vector3z                m_dcenter{};
  rmath::vector3z                m_danchor{};
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_UNICELL_ENTITY3D_HPP_ */
