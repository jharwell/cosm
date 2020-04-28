/**
 * \file arena_grid.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_DS_ARENA_GRID_HPP_
#define INCLUDE_COSM_DS_ARENA_GRID_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <mutex>
#include <tuple>

#include "rcppsw/ds/stacked_grid2D.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/ds/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds);

/**
 * \brief The types of the layers used by \ref arena_grid.
 */
using arena_layer_stack = std::tuple<cell2D>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class arena_grid
 * \ingroup ds
 *
 * \brief 2D grid of \ref cell2D objects containing the state of the geometrical
 * extent of the arena floor.
 */
class arena_grid : public rds::stacked_grid2D<arena_layer_stack> {
 public:
  using cell_type = cell2D;

  using view = rds::base_grid2D<ds::cell2D>::grid_view;
  using const_view = rds::base_grid2D<cds::cell2D>::const_grid_view;

  static constexpr const size_t kCell = 0;

  /**
   * \param resolution The arena resolution (i.e. what is the size of 1 cell in
   *                   the 2D grid).
   * \param x_max      Size in X of 2D grid.
   * \param y_max      Size in Y of 2D grid.
   *
   * The origin of the grid is in the lower left corner at (0,0).
   */
  arena_grid(const rmath::vector2d& dims,
             const rtypes::discretize_ratio& resolution)
      : stacked_grid2D(dims, resolution) {
    for (size_t i = 0; i < xdsize(); ++i) {
      for (size_t j = 0; j < ydsize(); ++j) {
        access<kCell>(i, j).loc(rmath::vector2z(i, j));
      } /* for(j..) */
    }   /* for(i..) */
  }

  /**
   * \brief Reset all the cells within the grid, removing all references to old
   * blocks as well as setting all cells back to an empty state.
   */
  void reset(void) {
    for (size_t i = 0; i < xdsize(); ++i) {
      for (size_t j = 0; j < ydsize(); ++j) {
        access<kCell>(i, j).reset();
      } /* for(j..) */
    }   /* for(i..) */
  }     /* reset */

  std::mutex* mtx(void) { return &m_mtx; }

 private:
  /* clang-format off */
  std::mutex m_mtx{};
  /* clang-format on */
};

NS_END(ds, cosm);

#endif /* INCLUDE_COSM_DS_ARENA_GRID_HPP_ */
