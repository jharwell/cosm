/**
 * \file grid2D_view_entity.hpp
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

#ifndef INCLUDE_COSM_REPR_GRID2D_VIEW_ENTITY_HPP_
#define INCLUDE_COSM_REPR_GRID2D_VIEW_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/repr/entity2D.hpp"
#include "cosm/repr/base_grid_view_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class grid2D_view_entity
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
class grid2D_view_entity : public crepr::entity2D,
                           public crepr::base_grid_view_entity<TGridType, TGridViewType>,
                           public rer::client<grid2D_view_entity<TGridType, TGridViewType>> {
 public:
  using base_grid_view_entity_type = base_grid_view_entity<TGridType, TGridViewType>;

  using typename base_grid_view_entity_type::grid_type;
  using typename base_grid_view_entity_type::grid_view_type;
  using typename base_grid_view_entity_type::cell_type;
  using typename base_grid_view_entity_type::coord_type;

  using base_grid_view_entity_type::resolution;
  using base_grid_view_entity_type::access;

  grid2D_view_entity(const rtypes::type_uuid& id,
                     const grid_view_type& the_view,
                     const rtypes::discretize_ratio& res)
      : entity2D(id),
      base_grid_view_entity_type(the_view, res),
      ER_CLIENT_INIT("cosm.repr.grid2D_view_entity") {}

  ~grid2D_view_entity(void) override = default;

  rmath::vector2d ranchor2D(void) const override {
    return rmath::zvec2dvec(danchor2D(), resolution().v());
  }
  rmath::vector2d rcenter2D(void) const override {
    return ranchor2D() + rmath::vector2d(xrsize().v(), yrsize().v()) / 2.0;
  }

  rmath::vector2z danchor2D(void) const override final {
    return view().origin()->loc();
  }
  rmath::vector2z dcenter2D(void) const override final {
    return danchor2D() + rmath::vector2z(xdsize(), ydsize()) / 2;
  }

  rmath::ranged xrspan(void) const override {
    return entity2D::xrspan(ranchor2D(), xrsize());
  }
  rmath::ranged yrspan(void) const override {
    return entity2D::yrspan(ranchor2D(), yrsize());
  }
  rmath::rangez xdspan(void) const override {
    return entity2D::xdspan(danchor2D(), xdsize());
  }
  rmath::rangez ydspan(void) const override {
    return entity2D::ydspan(danchor2D(), ydsize());
  }

  rtypes::spatial_dist xrsize(void) const override final {
    return rtypes::spatial_dist(xdsize() * resolution().v());
  }
  rtypes::spatial_dist yrsize(void) const override final {
    return rtypes::spatial_dist(ydsize() * resolution().v());
  }

  /**
   * \brief Get the cell associated with a particular grid location within the
   * view. Asserts that both coordinates are within the bounds of the grid
   * underlying the view.
   *
   * \param i The RELATIVE X coord within the view.
   * \param j The RELATIVE Y coord within the view.
   *
   * \return A reference to the cell.
   */
  const cell_type& access(size_t i, size_t j) const {
    ER_ASSERT(i < xdsize(), "Out of bounds X access: %zu >= %lu", i, xdsize());
    ER_ASSERT(j < ydsize(), "Out of bounds Y access: %zu >= %lu", j, ydsize());
    return view()[i][j];
  }

  const cell_type& access(const coord_type& c) const override {
    return access(c.x(), c.y());
  }

  bool contains_abs(const coord_type& cell) const override {
    return xdspan().contains(cell.x()) && ydspan().contains(cell.y());
  }

  bool contains_rel(const coord_type& cell) const override {
    return (cell.x() < xdsize()) && (cell.y() < ydsize());
  }

  size_t xdsize(void) const override final { return view().shape()[0]; }
  size_t ydsize(void) const override final { return view().shape()[1]; }

 protected:
  using base_grid_view_entity_type::view;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_GRID2D_VIEW_ENTITY_HPP_ */
