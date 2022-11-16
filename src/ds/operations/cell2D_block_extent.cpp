/**
 * \file cell2D_block_extent.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/operations/cell2D_block_extent.hpp"

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds, operations, detail);
using cads::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell2D_block_extent::cell2D_block_extent(const rmath::vector2z& coord,
                                         crepr::sim_block3D* block)
    : cell2D_op(coord), m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell2D_block_extent::visit(cds::cell2D& cell) {
  cell.entity(m_block);
  visit(cell.fsm());
  cell.color(m_block->md()->color());
} /* visit() */

void cell2D_block_extent::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_block_extent();
} /* visit() */

void cell2D_block_extent::visit(cads::arena_grid& grid) {
  visit(grid.access<arena_grid::kCell>(cell2D_op::coord()));
} /* visit() */

NS_END(detail, operations, ds, cosm);
