/**
 * \file stop.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::fsm {
class supervisor_fsm;
} /* namespace fsm */

namespace cosm::controller {
class base_controller;
}

NS_START(cosm, controller, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class stop
 * \ingroup controller operations
 *
 * \brief Fired whenever a robot has been told to/determined to stop doing stuff
 * indefinitely (until it is reset).
 *
 * We do not use the precise visitor, because) this is a super generic
 * operation. Implicit upcasting is OK is THIS SINGLE CASE.
 */
class stop : public rer::client<stop> {
 public:
  stop(void) : ER_CLIENT_INIT("cosm.controller.operations.stop") {}
  ~stop(void) override = default;

  stop(const stop&) = delete;
  stop& operator=(const stop&) = delete;

  void visit(cfsm::supervisor_fsm& fsm);
  void visit(ccontroller::base_controller& controller);
};

NS_END(operations, controller, cosm);

