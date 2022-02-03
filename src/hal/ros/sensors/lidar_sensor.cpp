/**
 * \file lidar_sensor.cpp
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
#include "cosm/hal/ros/sensors/lidar_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
lidar_sensor::lidar_sensor(const cros::topic& robot_ns)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.lidar"),
      ros_sensor(robot_ns) {
  enable();
}

lidar_sensor::lidar_sensor(lidar_sensor&& other)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.lidar_sensor"),
      ros_sensor(other.robot_ns()) {
  enable();
}

lidar_sensor& lidar_sensor::operator=(lidar_sensor&& rhs) {
  this->m_scan = rhs.m_scan;
  rhs.disable();
  this->enable();
  return *this;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void lidar_sensor::reset(void) { m_scan = {}; }

void lidar_sensor::enable(void) {
  if (is_enabled()) {
    return;
  }
  auto topic = robot_ns() / cros::topic(kScanTopic);
  ER_INFO("%s: ns=%s, topic=%s, subscribe=%s",
          __FUNCTION__,
          robot_ns().c_str(),
          kScanTopic.c_str(),
          topic.c_str());


  subscribe(topic, &lidar_sensor::callback, this);
}

std::vector<rmath::vector2d> lidar_sensor::readings(void) const {
  ER_ASSERT(is_enabled(),
            "%s called when disabled",
            __FUNCTION__);

  std::vector<rmath::vector2d> ret;

  for (size_t i = 0; i < m_scan.ranges.size(); ++i) {
    if (RCPPSW_IS_BETWEEN(m_scan.range_min,
                          m_scan.range_max,
                          m_scan.ranges[i])) {
      auto angle = rmath::radians(m_scan.angle_min +
                                  m_scan.angle_max * m_scan.angle_increment);
      ret.push_back(rmath::vector2d(m_scan.ranges[i], angle));
    }
  } /* for(i..) */

  return ret;
}

NS_END(sensors, ros, hal, cosm);
