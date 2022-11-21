/**
 * \file ros_service_sensor.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/ros/sensors/ros_service_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::ros::sensors {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ros_service_sensor::ros_service_sensor(const cros::topic& robot_ns)
    : ER_CLIENT_INIT("cos.hal.ros.sensors.ros_service_sensor"),
      m_robot_ns(robot_ns) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ros_service_sensor::disable(void) {
  if (is_enabled()) {
    ER_DEBUG("Disconnecting from service '%s'", m_name.c_str());
    decoratee().~impl_type();
    m_name = "";
  }
}

} /* namespace cosm::hal::ros::sensors */
