/**
 * \file stop.cpp
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
#include "cosm/controller/operations/stop.hpp"

#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/pal/argos_controller2D_adaptor.hpp"
#include "cosm/pal/argos_controllerQ3D_adaptor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, controller, operations);

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void stop::visit(ccontroller::base_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.supervisor());

  ER_INFO("Robot%d stopped", controller.entity_id().v());
  controller.ndc_pop();
} /* visit() */

void stop::visit(cfsm::supervisor_fsm& fsm) {
  fsm.event_stop();
} /* visit() */

NS_END(operations, controller, cosm);
