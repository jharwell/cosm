/**
 * \file robot_malfunction.hpp
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

#ifndef INCLUDE_COSM_PAL_ARGOS_OPERATIONS_ROBOT_MALFUNCTION_HPP_
#define INCLUDE_COSM_PAL_ARGOS_OPERATIONS_ROBOT_MALFUNCTION_HPP_

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

NS_START(cosm, pal, argos, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_malfunction
 * \ingroup pal argos operations
 *
 * \brief Fired whenever a robot is determined to have mechanically
 * malfunctioned by the loop functions.
 *
 * We do not use the precise visitor, because (1) this is a super generic
 * operation, and (2) doing so makes this not able to be used in the \ref
 * pd_adaptor, which is what it was designed for in the first
 * place. Implicit upcasting is OK is THIS SINGLE CASE.
 */
class robot_malfunction : public rer::client<robot_malfunction> {
 public:
  robot_malfunction(void) :
      ER_CLIENT_INIT("cosm.pal.argos.operations.robot_malfunction") {}
  ~robot_malfunction(void) override = default;

  robot_malfunction(const robot_malfunction&) = delete;
  robot_malfunction& operator=(const robot_malfunction&) = delete;

  void visit(cfsm::supervisor_fsm& fsm);
  void visit(cpargos::controller::adaptor2D& controller);
  void visit(cpargos::controller::adaptorQ3D& controller);
};

NS_END(operations, argos, pal, cosm);

#endif /* INCLUDE_COSM_PAL_ARGOS_OPERATIONS_ROBOT_MALFUNCTION_HPP_ */
