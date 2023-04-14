/**
 * \file irv_recipient_controller.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::tv {
class robot_dynamics_applicator;
} /* namespace tv */

namespace cosm::controller {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class irv_recipient_controller
 * \ingroup controller
 *
 * \brief Defines the interface for all Internal Robot Variance (IRV) accepting
 * controllers that can have temporal variance applied to their internal state,
 * as distinct from the variance that can be applied when the robot interacts
 * with the environment.
 */
class irv_recipient_controller {
 public:
  irv_recipient_controller(void) = default;
  virtual ~irv_recipient_controller(void) = default;

  /**
   * \brief Return the applied movement throttling for the robot. This is not
   * necessarily the same as the active/configured throttling.
   */
  virtual double applied_movement_throttle(void) const = 0;

  /**
   * \brief Perform necessary initializations to register the controller with
   * the provided \ref robot_dynamics_applicator, types of variances to apply,
   * etc, as configured.
   */
  virtual void irv_init(const tv::robot_dynamics_applicator* irv) = 0;
};

} /* namespace cosm::controller */
