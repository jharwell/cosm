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
  auto n_publishers = decoratee().getNumPublishers();

  ER_ASSERT(1 == n_publishers,
            "Expected 1 publisher of lidar data for %s, got %d",
            cpal::kRobotType.c_str(),
            n_publishers);
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

  /*
   * Even though we get way more readings than from the IR sensors for the ARGoS
   * foot-bot, we don't want to downsample, because that will leave blindspots
   * which will make collision detection/obstacle avoidance much less reliable.
   */
  for (size_t i = 0; i < m_scan.ranges.size(); ++i) {
    if (RCPPSW_IS_BETWEEN(m_scan.ranges[i],
                          m_scan.range_min,
                          m_scan.range_max)) {
      auto angle = rmath::radians(m_scan.angle_min +
                                  i * m_scan.angle_increment);

      /*
       * To be consistent with what we have to use with ARGoS. Not ideal, but
       * it makes the collision avoidance code common between ROS and ARGoS.
       */
      auto dist = std::exp(-m_scan.ranges[i]);
      ret.push_back(rmath::vector2d(dist, angle));
    }
  } /* for(i..) */

  return ret;
}

void lidar_sensor::callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  m_scan = *msg;
}

NS_END(sensors, ros, hal, cosm);
