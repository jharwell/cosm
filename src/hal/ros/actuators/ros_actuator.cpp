/**
 * \file ros_actuator.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/ros/actuators/ros_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::ros::actuators {

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

} /* namespace cosm::hal::ros::actuators */
