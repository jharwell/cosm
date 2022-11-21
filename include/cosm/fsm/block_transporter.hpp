/**
 * \file block_transporter.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::fsm {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_transporter
 * \ingroup fsm
 *
 * \brief Interface defining what classes directly involved in transporting
 * blocks need to implement in order to successfully interact with the loop
 * functions.
 */
template <typename TGoal>
class block_transporter {
 public:
  block_transporter(void) = default;
  virtual ~block_transporter(void) = default;

  /**
   * \brief All tasks must define method to determine what they are currently
   * doing with a block (if they are carrying one).
   */
  virtual TGoal block_transport_goal(void) const = 0;
};

} /* namespace cosm::fsm */
