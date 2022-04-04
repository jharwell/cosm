/**
 * \file ros_sensor.cpp
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
#include "cosm/hal/ros/sensors/ros_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ros_sensor::ros_sensor(const cros::topic& robot_ns)
    : ER_CLIENT_INIT("cos.hal.ros.sensors.ros_sensor"),
      m_robot_ns(robot_ns) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ros_sensor::disable(void) {
  if (is_enabled()) {
    ER_DEBUG("Remove subscription to '%s'", m_topic.c_str());
    decoratee().~impl_type();
    m_topic = "";
  }
}

NS_END(sensors, ros, hal, cosm);
