/**
 * \file robot_repair.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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

namespace cosm::pal::argos::controller {
class adaptor2D;
class adaptorQ3D;
} /* namespace cosm::pal */

NS_START(cosm, argos, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_repair
 * \ingroup argos operations
 *
 * \brief Fired whenever a robot is determined to have had a mechanical
 * malfunction repaired by the loop functions.
 *
 * We do not use the precise visitor, because (1) this is a super generic
 * operation, and (2) doing so makes this not able to be used in the \ref
 * pd_adaptor, which is what it was designed for in the first
 * time. Implicit upcasting is OK is THIS SINGLE CASE.
 */
class robot_repair : public rer::client<robot_repair> {
 public:
  robot_repair(void)
      : ER_CLIENT_INIT("cosm.pal.argos.operations.robot_repair") {}
  ~robot_repair(void) override = default;

  robot_repair(const robot_repair& op) = delete;
  robot_repair& operator=(const robot_repair& op) = delete;

  void visit(cfsm::supervisor_fsm& fsm);
  void visit(cpargos::controller::adaptor2D& controller);
  void visit(cpargos::controller::adaptorQ3D& controller);
};

NS_END(operations, argos, cosm);

