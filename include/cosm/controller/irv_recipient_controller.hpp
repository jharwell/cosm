/**
 * \file irv_recipient_controller.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_CONTROLLER_IRV_RECIPIENT_CONTROLLER_HPP_
#define INCLUDE_COSM_CONTROLLER_IRV_RECIPIENT_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm);

namespace tv {
class swarm_irv_manager;
} /* namespace tv */

NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class irv_recipient_controller
 * \ingroup controller
 *
 * \brief Internal Temporal Variance (IRV) recipient controller. Defines the
 * interface for all controllers that can have temporal variance applied to
 * their internal state, as distinct from the variance that can be applied when
 * the robot interacts with the environment.
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
   * the provided \ref swarm_irv_manager, types of variances to apply, etc, as
   * configured.
   */
  virtual void irv_init(const tv::swarm_irv_manager* irv_manager) = 0;
};

NS_END(controller, cosm);

#endif /* INCLUDE_COSM_CONTROLLER_IRV_RECIPIENT_CONTROLLER_HPP_ */
