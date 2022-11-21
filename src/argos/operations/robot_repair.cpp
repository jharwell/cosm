/**
 * \file robot_repair.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::argos::operations {

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

} /* namespace cosm::argos::operations */
