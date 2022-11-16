/**
 * \file stop.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/operations/stop.hpp"

#include "cosm/controller/base_controller.hpp"
#include "cosm/fsm/supervisor_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, controller, operations);

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void stop::visit(ccontroller::base_controller& controller) {
  controller.ndc_uuid_push();

  visit(*controller.supervisor());

  ER_INFO("Robot%d stopped", controller.entity_id().v());
  controller.ndc_uuid_pop();
} /* visit() */

void stop::visit(cfsm::supervisor_fsm& fsm) { fsm.event_stop(); } /* visit() */

NS_END(operations, controller, cosm);
