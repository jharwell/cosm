/**
 * \file cell2D_cache_extent.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/operations/cell2D_cache_extent.hpp"

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/ds/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds::operations::detail {
using cads::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell2D_cache_extent::cell2D_cache_extent(const rmath::vector2z& coord,
                                         carepr::base_cache* cache)
    : cell2D_op(coord), m_cache(cache) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell2D_cache_extent::visit(cds::cell2D& cell) {
  cell.entity(m_cache);
  visit(cell.fsm());
  cell.color(m_cache->color());
} /* visit() */

void cell2D_cache_extent::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_cache_extent();
} /* visit() */

void cell2D_cache_extent::visit(cads::arena_grid& grid) {
  visit(grid.access<arena_grid::kCell>(cell2D_op::coord()));
} /* visit() */

} /* namespace cosm::ds::operations::detail */
