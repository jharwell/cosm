/**
 * \file cell2D_unknown.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/operations/cell2D_unknown.hpp"

#include "cosm/ds/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds::operations {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell2D_unknown::visit(cds::cell2D& cell) {
  cell.entity(nullptr);
  visit(cell.fsm());
} /* visit() */

void cell2D_unknown::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_unknown();
} /* visit() */

} /* namespace cosm::ds::operations */
