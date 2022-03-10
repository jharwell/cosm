/**
 * \file light_sensor.cpp
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
#include "cosm/hal/ros/sensors/light_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);


/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
light_sensor::light_sensor(const cros::topic& robot_ns)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.light"),
      ros_sensor(robot_ns) {
  enable();
  auto n_publishers = decoratee().getNumPublishers();

  ER_ASSERT(1 == n_publishers,
            "Expected 1 publisher of light data for %s, got %d",
            cpal::kRobotType.c_str(),
            n_publishers);
}

light_sensor::light_sensor(light_sensor&& other)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.light_sensor"),
      ros_sensor(other.robot_ns()) {
  enable();
}

light_sensor& light_sensor::operator=(light_sensor&& rhs) {
  this->m_light = rhs.m_light;
  rhs.disable();
  this->enable();
  return *this;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void light_sensor::reset(void) { m_light = {}; }

void light_sensor::enable(void) {
  if (is_enabled()) {
    return;
  }
  auto topic = robot_ns() / cros::topic(kLightTopic);
  ER_INFO("%s: ns=%s, topic=%s, subscribe=%s",
          __FUNCTION__,
          robot_ns().c_str(),
          kLightTopic.c_str(),
          topic.c_str());


  subscribe(topic, &light_sensor::callback, this);
}

std::vector<light_sensor::reading_type> light_sensor::readings(void) const {
  ER_ASSERT(is_enabled(),
            "%s called when disabled",
            __FUNCTION__);
  return {};
}

void light_sensor::callback(const std_msgs::Float32::ConstPtr& msg) {
  m_light = *msg;
}

NS_END(sensors, ros, hal, cosm);
