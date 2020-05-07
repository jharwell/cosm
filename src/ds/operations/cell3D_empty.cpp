/**
 * \file cell3D_empty.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/ds/operations/cell3D_empty.hpp"

#include "cosm/ds/cell3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds, operations);

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

NS_END(operations, ds, cosm);
