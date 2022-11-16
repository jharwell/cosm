/**
 * \file ros_subscriber_sensor.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/ros/sensors/ros_subscriber_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ros_subscriber_sensor::ros_subscriber_sensor(const cros::topic& robot_ns)
    : ER_CLIENT_INIT("cos.hal.ros.sensors.ros_subscriber_sensor"),
      m_robot_ns(robot_ns) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ros_subscriber_sensor::disable(void) {
  if (is_enabled()) {
    ER_DEBUG("Remove subscription to '%s'", m_topic.c_str());
    decoratee().~impl_type();
    m_topic = "";
  }
}

NS_END(sensors, ros, hal, cosm);
