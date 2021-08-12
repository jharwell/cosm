/**
 * \file stop.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_CONTROLLER_OPERATIONS_STOP_HPP_
#define INCLUDE_COSM_CONTROLLER_OPERATIONS_STOP_HPP_

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
  stop(void) : ER_CLIENT_INIT("cosm.events.stop") {}
  ~stop(void) override = default;

  stop(const stop&) = delete;
  stop& operator=(const stop&) = delete;

  void visit(cfsm::supervisor_fsm& fsm);
  void visit(ccontroller::base_controller& controller);
};

NS_END(operations, controller, cosm);

#endif /* INCLUDE_COSM_CONTROLLER_OPERATIONS_STOP_HPP_ */
