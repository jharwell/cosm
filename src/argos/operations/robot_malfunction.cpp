/**
 * \file robot_malfunction.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/argos/operations/robot_malfunction.hpp"

#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/pal/argos/controller/adaptor2D.hpp"
#include "cosm/pal/argos/controller/adaptorQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, argos, operations);

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void robot_malfunction::visit(cpargos::controller::adaptor2D& controller) {
  controller.ndc_uuid_push();

  visit(*controller.supervisor());

  ER_INFO("Robot %s malfunctioned", controller.GetId().c_str());
  controller.ndc_uuid_pop();
} /* visit() */

void robot_malfunction::visit(cpargos::controller::adaptorQ3D& controller) {
  controller.ndc_uuid_push();

  visit(*controller.supervisor());

  ER_INFO("Robot %s malfunctioned", controller.GetId().c_str());
  controller.ndc_uuid_pop();
} /* visit() */

void robot_malfunction::visit(cfsm::supervisor_fsm& fsm) {
  fsm.event_malfunction();
} /* visit() */

NS_END(operations, argos, cosm);
