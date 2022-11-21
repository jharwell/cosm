/**
 * \file grid3D_view_entity.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/base_grid_view_entity.hpp"
#include "cosm/repr/entity3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class grid3D_view_entity
 * \ingroup repr
 *
 * \brief Representation of an \ref cads::arena_grid::view or \ref
 * cads::arena_grid::const_view object as an entity in the arena. This
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
class grid3D_view_entity
    : public crepr::entity3D,
      public crepr::base_grid_view_entity<TGridType, TGridViewType>,
      public rer::client<grid3D_view_entity<TGridType, TGridViewType>> {
 public:
  using grid_view_entity_type = base_grid_view_entity<TGridType, TGridViewType>;

  using typename grid_view_entity_type::cell_type;
  using typename grid_view_entity_type::coord_type;
  using typename grid_view_entity_type::grid_type;
  using typename grid_view_entity_type::grid_view_type;

  using grid_view_entity_type::access;
  using grid_view_entity_type::resolution;

  grid3D_view_entity(const rtypes::type_uuid& id,
                     const grid_view_type& the_view,
                     const rtypes::discretize_ratio& res)
      : entity3D(id,
                 rmath::vector3z(the_view.shape()[0],
                                 the_view.shape()[1],
                                 the_view.shape()[2]),
                 the_view.origin()->loc(),
                 rspatial::euclidean_dist(res.v())),
        grid_view_entity_type(the_view, res),
        ER_CLIENT_INIT("cosm.repr.grid3D_view_entity") {}

  ~grid3D_view_entity(void) override = default;

  const cell_type& access(size_t i, size_t j, size_t k) const {
    ER_ASSERT(i < xdsize(), "Out of bounds X access: %zu >= %zu", i, xdsize());
    ER_ASSERT(j < ydsize(), "Out of bounds Y access: %zu >= %zu", j, ydsize());
    ER_ASSERT(k < ydsize(), "Out of bounds Z access: %zu >= %zu", k, zdsize());
    return view()[i][j][k];
  }

  const cell_type& access(const coord_type& c) const override {
    return access(c.x(), c.y(), c.z());
  }

  bool contains_abs(const coord_type& cell) const override {
    return xdspan().contains(cell.x()) && ydspan().contains(cell.y()) &&
           zdspan().contains(cell.z());
  }
  bool contains_rel(const coord_type& cell) const override {
    return (cell.x() < xdsize()) && (cell.y() < ydsize()) &&
           (cell.z() < zdsize());
  }

 protected:
  using grid_view_entity_type::view;
};

} /* namespace cosm::repr */
