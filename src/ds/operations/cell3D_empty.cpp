/**
 * \file cell3D_empty.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/operations/cell3D_empty.hpp"

#include "cosm/ds/cell3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds::operations {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell3D_empty::visit(cds::cell3D& cell) {
  cell.entity(nullptr);
  visit(cell.fsm());
} /* visit() */

void cell3D_empty::visit(fsm::cell3D_fsm& fsm) {
  fsm.event_empty();
} /* visit() */

} /* namespace cosm::ds::operations */
