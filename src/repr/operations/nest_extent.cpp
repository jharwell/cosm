/**
 * \file nest_extent.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/operations/nest_extent.hpp"

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/nest.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr::operations::detail {
using cads::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_extent::nest_extent(const rmath::vector2z& coord, crepr::nest* nest)
    : cell2D_op(coord), m_nest(nest) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_extent::visit(cds::cell2D& cell) {
  cell.entity(m_nest);
  visit(cell.fsm());
  cell.color(m_nest->color());
} /* visit() */

void nest_extent::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_nest_extent();
} /* visit() */

void nest_extent::visit(cads::arena_grid& grid) {
  visit(grid.access<arena_grid::kCell>(cell2D_op::coord()));
} /* visit() */

} /* namespace cosm::repr::operations::detail */
