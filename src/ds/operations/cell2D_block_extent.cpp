/**
 * \file cell2D_block_extent.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/operations/cell2D_block_extent.hpp"

#include "cosm/ds/arena_grid.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds, operations, detail);
using cds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell2D_block_extent::cell2D_block_extent(const rmath::vector2z& coord,
                                         crepr::base_block3D* block)
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

void cell2D_block_extent::visit(cds::arena_grid& grid) {
  visit(grid.access<arena_grid::kCell>(cell2D_op::coord()));
} /* visit() */

NS_END(detail, operations, ds, cosm);
