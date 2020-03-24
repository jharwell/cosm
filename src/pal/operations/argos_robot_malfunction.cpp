/**
 * \file argos_robot_malfunction.cpp
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
#include "cosm/pal/operations/argos_robot_malfunction.hpp"
#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/pal/argos_controller2D_adaptor.hpp"
#include "cosm/pal/argos_controllerQ3D_adaptor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, operations);

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void argos_robot_malfunction::visit(cpal::argos_controller2D_adaptor& controller) {
  controller.ndc_pusht();

  visit(*controller.supervisor());

  ER_INFO("Robot %s malfunctioned", controller.GetId().c_str());
  controller.ndc_pop();
} /* visit() */

void argos_robot_malfunction::visit(cpal::argos_controllerQ3D_adaptor& controller) {
  controller.ndc_pusht();

  visit(*controller.supervisor());

  ER_INFO("Robot %s malfunctioned", controller.GetId().c_str());
  controller.ndc_pop();
} /* visit() */

void argos_robot_malfunction::visit(cfsm::supervisor_fsm& fsm) {
  fsm.event_malfunction();
} /* visit() */

NS_END(operations, pal, cosm);
