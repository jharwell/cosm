/**
 * \file gridQ3D_view_entity.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/entity3D.hpp"
#include "cosm/repr/grid3D_view_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {

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
class gridQ3D_view_entity
    : public rer::client<gridQ3D_view_entity<TGridType, TGridViewType>>,
      public crepr::entity3D,
      public crepr::base_grid_view_entity<TGridType, TGridViewType> {
 public:
  using grid_view_entity_type = base_grid_view_entity<TGridType, TGridViewType>;

  using typename grid_view_entity_type::cell_type;
  using typename grid_view_entity_type::coord_type;
  using typename grid_view_entity_type::grid_type;
  using typename grid_view_entity_type::grid_view_type;

  using grid_view_entity_type::resolution;

  gridQ3D_view_entity(const rtypes::type_uuid& id,
                      const grid_view_type& the_view,
                      size_t zdsize,
                      const rtypes::discretize_ratio& res)
      : ER_CLIENT_INIT("cosm.repr.gridQ3D_view_entity"),
        entity3D(id,
                 rmath::vector3z({ the_view.shape()[0], the_view.shape()[1] },
                                 zdsize),
                 rmath::vector3z(the_view.origin()->loc(), 0),
                 rspatial::euclidean_dist(res.v())),
        grid_view_entity_type(the_view, res) {}

  ~gridQ3D_view_entity(void) override = default;

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

 protected:
  using grid_view_entity_type::view;
};

} /* namespace cosm::repr */
