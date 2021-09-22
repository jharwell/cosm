/**
 * \file argos_robot_repair.hpp
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

#ifndef INCLUDE_COSM_PAL_OPERATIONS_ARGOS_ROBOT_REPAIR_HPP_
#define INCLUDE_COSM_PAL_OPERATIONS_ARGOS_ROBOT_REPAIR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::fsm {
class supervisor_fsm;
} /* namespace fsm */

namespace cosm::pal {
class argos_controller2D_adaptor;
class argos_controllerQ3D_adaptor;
} /* namespace cosm::pal */

NS_START(cosm, pal, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class argos_robot_repair
 * \ingroup pal operations detail
 *
 * \brief Fired whenever a robot is determined to have had a mechanical
 * malfunction repaired by the loop functions.
 *
 * We do not use the precise visitor, because (1) this is a super generic
 * operation, and (2) doing so makes this not able to be used in the \ref
 * argos_pd_adaptor, which is what it was designed for in the first
 * time. Implicit upcasting is OK is THIS SINGLE CASE.
 */
class argos_robot_repair : public rer::client<argos_robot_repair> {
 public:
  argos_robot_repair(void) : ER_CLIENT_INIT("cosm.events.argos_robot_repair") {}
  ~argos_robot_repair(void) override = default;

  argos_robot_repair(const argos_robot_repair& op) = delete;
  argos_robot_repair& operator=(const argos_robot_repair& op) = delete;

  void visit(cfsm::supervisor_fsm& fsm);
  void visit(cpal::argos_controller2D_adaptor& controller);
  void visit(cpal::argos_controllerQ3D_adaptor& controller);
};

NS_END(operations, pal, cosm);

#endif /* INCLUDE_COSM_PAL_OPERATIONS_ARGOS_ROBOT_REPAIR_HPP_ */
