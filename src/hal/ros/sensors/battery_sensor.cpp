/**
 * \file battery_sensor.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/ros/sensors/battery_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
battery_sensor::battery_sensor(const cros::topic& robot_ns)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.battery"),
      ros_subscriber_sensor(robot_ns) {
  enable();
  auto n_publishers = decoratee().getNumPublishers();

  ER_ASSERT(1 == n_publishers,
            "Expected 1 publisher of battery data for %s, got %d",
            cpal::kRobotType.c_str(),
            n_publishers);
}

battery_sensor::battery_sensor(battery_sensor&& other)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.battery"),
      ros_subscriber_sensor(other.robot_ns()) {
  enable();
}

battery_sensor& battery_sensor::operator=(battery_sensor&& rhs) {
  this->m_state = rhs.m_state;
  rhs.disable();
  this->enable();
  return *this;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void battery_sensor::reset(void) { m_state = {}; }

void battery_sensor::enable(void) {
  if (is_enabled()) {
    return;
  }
  auto topic = robot_ns() / cros::topic(kTopic);
  ER_INFO("%s: ns=%s, topic=%s, subscribe=%s",
          __FUNCTION__,
          robot_ns().c_str(),
          kTopic.c_str(),
          topic.c_str());

  subscribe(topic, &battery_sensor::callback, this);
}

std::vector<chsensors::battery_sensor_reading> battery_sensor::readings(void) const {
  ER_ASSERT(is_enabled(), "%s called when disabled", __FUNCTION__);

  chal::sensors::battery_sensor_reading ret;
  ret.voltage =  m_state.voltage;
  ret.percentage = m_state.percentage;

  return {ret};
}

void battery_sensor::callback(const sensor_msgs::BatteryState::ConstPtr& msg) {
  m_state = *msg;
}

/*******************************************************************************
 * Battery Metrics
 ******************************************************************************/
double battery_sensor::percent_remaining(void) const {
  return readings()[0].percentage;
} /* percent_remaining() */

NS_END(sensors, ros, hal, cosm);
