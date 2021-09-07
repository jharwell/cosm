/**
 * \file grid3D_view_entity.hpp
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

#ifndef INCLUDE_COSM_REPR_GRID3D_VIEW_ENTITY_HPP_
#define INCLUDE_COSM_REPR_GRID3D_VIEW_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/entity3D.hpp"
#include "cosm/repr/base_grid_view_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class grid3D_view_entity
 * \ingroup repr
 *
 * \brief Representation of an \ref arena_grid::grid_view or \ref
 * arena_grid::const_grid_view object as an entity in the arena. This
 * representation makes it much easier to pass abstract, mult-cell objects that
 * are not entities per-se as entities for various operations such as block
 * distribution in foraging.
 *
 * It has the following characteristics:
 *
 * - Spans multiple cells in the arena.
 * - Does not "exist" in the sense that it is not detectable by robots. It lives
 *   on the same level of abstraction as the arena grid (hence the class name).
 * - Has no concept of movability/immovability (again, it is abstract).
 */
template <typename TGridType, typename TGridViewType>
class grid3D_view_entity : public crepr::entity3D,
                           public crepr::base_grid_view_entity<TGridType, TGridViewType>,
                           public rer::client<grid3D_view_entity<TGridType, TGridViewType>> {
 public:
  using grid_view_entity_type = base_grid_view_entity<TGridType, TGridViewType>;

  using typename grid_view_entity_type::grid_type;
  using typename grid_view_entity_type::grid_view_type;
  using typename grid_view_entity_type::cell_type;
  using typename grid_view_entity_type::coord_type;

  using grid_view_entity_type::resolution;
  using grid_view_entity_type::access;

  grid3D_view_entity(const rtypes::type_uuid& id,
                     const grid_view_type& the_view,
                     const rtypes::discretize_ratio& res)
      : entity3D(id),
        grid_view_entity_type(the_view, res),
        ER_CLIENT_INIT("cosm.repr.grid3D_view_entity") {}

  ~grid3D_view_entity(void) override = default;


  rmath::vector3d ranchor3D(void) const override {
    return rmath::zvec2dvec(danchor3D(), resolution().v());
  }
  rmath::vector3d rcenter3D(void) const override {
    return ranchor3D() + rmath::vector3d(xrsize().v(),
                                         yrsize().v(),
                                         zrsize().v()) / 2.0;
  }

  coord_type danchor3D(void) const override final {
    return view().origin()->loc();
  }
  coord_type dcenter3D(void) const override final {
    return danchor3D() + coord_type(xdsize(),
                                    ydsize(),
                                    zdsize()) / 2;
  }


  rmath::ranged xrspan(void) const override {
    return entity3D::xrspan(ranchor3D(), xrsize());
  }
  rmath::ranged yrspan(void) const override {
    return entity3D::yrspan(ranchor3D(), yrsize());
  }
  rmath::ranged zrspan(void) const override {
    return entity3D::zrspan(ranchor3D(), yrsize());
  }

  rmath::rangez xdspan(void) const override {
    return entity3D::xdspan(danchor3D(), xdsize());
  }
  rmath::rangez ydspan(void) const override {
    return entity3D::ydspan(danchor3D(), ydsize());
  }
  rmath::rangez zdspan(void) const override {
    return entity3D::zdspan(danchor3D(), zdsize());
  }

  rtypes::spatial_dist xrsize(void) const override final {
    return rtypes::spatial_dist(xdsize() * resolution().v());
  }
  rtypes::spatial_dist yrsize(void) const override final {
    return rtypes::spatial_dist(ydsize() * resolution().v());
  }
  rtypes::spatial_dist zrsize(void) const override final {
    return rtypes::spatial_dist(zdsize() * resolution().v());
  }

  const cell_type& access(size_t i, size_t j, size_t k) const {
    ER_ASSERT(i < xdsize(), "Out of bounds X access: %zu >= %lu", i, xdsize());
    ER_ASSERT(j < ydsize(), "Out of bounds Y access: %zu >= %lu", j, ydsize());
    ER_ASSERT(k < ydsize(), "Out of bounds Z access: %zu >= %lu", k, zdsize());
    return view()[i][j][k];
  }

  const cell_type& access(const coord_type& c) const override {
    return access(c.x(), c.y(), c.z());
  }

  bool contains_abs(const coord_type& cell) const override {
    return xdspan().contains(cell.x()) &&
        ydspan().contains(cell.y()) &&
        zdspan().contains(cell.z());
  }
  bool contains_rel(const coord_type& cell) const override {
    return (cell.x() < xdsize()) &&
        (cell.y() < ydsize()) &&
        (cell.z() < zdsize());
  }

  size_t xdsize(void) const override final { return view().shape()[0]; }
  size_t ydsize(void) const override final { return view().shape()[1]; }
  size_t zdsize(void) const override final { return view().shape()[2]; }

 protected:
  using grid_view_entity_type::view;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_GRID3D_VIEW_ENTITY_HPP_ */
