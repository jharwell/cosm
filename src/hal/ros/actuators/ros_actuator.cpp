/**
 * \file ros_actuator.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include "cosm/hal/ros/actuators/ros_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, actuators);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ros_actuator::ros_actuator(const cros::topic& robot_ns)
    : ER_CLIENT_INIT("cosm.hal.ros.actuators.ros_actuator"),
      m_robot_ns(robot_ns) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ros_actuator::disable(void) {
  if (m_publishing) {
    decoratee().~impl_type();
    m_publishing = false;
  }
}

NS_END(actuators, ros, hal, cosm);
