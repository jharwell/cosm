/**
 * \file robot_repair.cpp
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
#include "cosm/argos/operations/robot_repair.hpp"

#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/pal/argos/controller/adaptor2D.hpp"
#include "cosm/pal/argos/controller/adaptorQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, argos, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void robot_repair::visit(cpargos::controller::adaptor2D& controller) {
  controller.ndc_uuid_push();

  visit(*controller.supervisor());

  ER_INFO("Robot %s repaired", controller.GetId().c_str());
  controller.ndc_uuid_pop();
} /* visit() */

void robot_repair::visit(cpargos::controller::adaptorQ3D& controller) {
  controller.ndc_uuid_push();

  visit(*controller.supervisor());

  ER_INFO("Robot %s repaired", controller.GetId().c_str());
  controller.ndc_uuid_pop();
} /* visit() */

void robot_repair::visit(cfsm::supervisor_fsm& fsm) {
  fsm.event_repair();
} /* visit() */

NS_END(operations, argos, cosm);
