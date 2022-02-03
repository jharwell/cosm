 /**
 * \file diagnostic_actuator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_ROS_ACTUATORS_DIAGNOSTIC_ACTUATOR_HPP_
#define INCLUDE_COSM_HAL_ROS_ACTUATORS_DIAGNOSTIC_ACTUATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/utils/color.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/actuators/diagnostics.hpp"
#include "cosm/hal/ros/actuators/ros_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, actuators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class diagnostic_actuator
 * \ingroup hal ros actuators
 *
 * \brief Diagnostic actuator.
 *
 *  Supports the following robots:
 *
 * - ROS turtlebot3 (stub until this is implemented in hardware).
 */
class diagnostic_actuator final : public rer::client<diagnostic_actuator>,
                                  public chros::actuators::ros_actuator {
 public:

  explicit diagnostic_actuator(bool enable)
      : ER_CLIENT_INIT("cosm.hal.ros.actuators.diagnostic"),
        ros_actuator(cros::topic()) {}

  /* copy constructible/assignable to work with the saa subsystem */
  diagnostic_actuator(const diagnostic_actuator&) = delete;
  diagnostic_actuator& operator=(const diagnostic_actuator&)= delete;
  diagnostic_actuator(diagnostic_actuator&&) = default;
  diagnostic_actuator& operator=(diagnostic_actuator&&)= default;

  void reset(void) override {}
  void enable(void) override {}

  void emit(const chactuators::diagnostics& type) {
    switch (type) {
      case chactuators::diagnostics::ekEXPLORE:
        break;
      case chactuators::diagnostics::ekSUCCESS:
        break;
      case chactuators::diagnostics::ekLEAVING_NEST:
        break;
      case chactuators::diagnostics::ekWAIT_FOR_SIGNAL:
        break;
      case chactuators::diagnostics::ekVECTOR_TO_GOAL:
        break;
      case chactuators::diagnostics::ekEXP_INTERFERENCE:
        break;
      default:
        ER_FATAL_SENTINEL("Unknown diagnostic category %d",
                          rcppsw::as_underlying(type));
    } /* switch() */
  }
};

NS_END(actuators, ros, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ROS_ACTUATORS_DIAGNOSTIC_ACTUATOR_HPP_ */
