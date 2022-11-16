/**
 * \file cell2D_empty.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/operations/cell2D_empty.hpp"

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/ds/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds, operations);
using cads::arena_grid;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell2D_empty::visit(cds::cell2D& cell) {
  cell.entity(nullptr);
  visit(cell.fsm());
  cell.color(rutils::color::kWHITE);
} /* visit() */

void cell2D_empty::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_empty();
} /* visit() */

void cell2D_empty::visit(cads::arena_grid& grid) {
  visit(grid.access<arena_grid::kCell>(coord()));
} /* visit() */

NS_END(operations, ds, cosm);
