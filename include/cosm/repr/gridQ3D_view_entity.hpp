/**
 * \file gridQ3D_view_entity.hpp
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

#ifndef INCLUDE_COSM_REPR_GRIDQ3D_VIEW_ENTITY_HPP_
#define INCLUDE_COSM_REPR_GRIDQ3D_VIEW_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/entity3D.hpp"
#include "cosm/repr/grid3D_view_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class gridQ3D_view_entity
 * \ingroup repr
 *
 * \brief Representation of an entity that lives on 2D grid, but that has an
 * imaginary Z dimension (i.e., it is a bounding box).
 */
template <typename TGridType, typename TGridViewType>
class gridQ3D_view_entity : public rer::client<gridQ3D_view_entity<TGridType,
                                                                   TGridViewType>>,
                            public crepr::entity3D,
                            public crepr::base_grid_view_entity<TGridType,
                                                                TGridViewType> {
 public:
  using grid_view_entity_type = base_grid_view_entity<TGridType, TGridViewType>;

  using typename grid_view_entity_type::grid_type;
  using typename grid_view_entity_type::grid_view_type;
  using typename grid_view_entity_type::cell_type;
  using typename grid_view_entity_type::coord_type;

  using grid_view_entity_type::resolution;

  gridQ3D_view_entity(const rtypes::type_uuid& id,
                      const grid_view_type& the_view,
                      size_t zdsize,
                      const rtypes::discretize_ratio& res)
      : ER_CLIENT_INIT("cosm.repr.gridQ3D_view_entity"),
        entity3D(id),
        grid_view_entity_type(the_view, res),
        mc_zdsize(zdsize) {}

  ~gridQ3D_view_entity(void) override = default;

  rmath::vector3d ranchor3D(void) const override {
    return rmath::zvec2dvec(danchor3D(), resolution().v());
  }
  rmath::vector3d rcenter3D(void) const override {
    return ranchor3D() + rmath::vector3d(xrsize().v(),
                                         yrsize().v(),
                                         zrsize().v()) / 2.0;
  }

  rmath::vector3z danchor3D(void) const override final {
    return rmath::vector3z(view().origin()->loc());
  }
  rmath::vector3z dcenter3D(void) const override final {
    return danchor3D() + rmath::vector3z(xdsize(),
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
    return entity3D::zrspan(ranchor3D(), zrsize());
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
  size_t zdsize(void) const override final { return mc_zdsize; }

 protected:
  using grid_view_entity_type::view;

 private:
  /* clang-format off */
  const size_t mc_zdsize;
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_GRIDQ3D_VIEW_ENTITY_HPP_ */
