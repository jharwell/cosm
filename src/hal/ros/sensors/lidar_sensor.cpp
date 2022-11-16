/**
 * \file lidar_sensor.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
lidar_sensor::lidar_sensor(const cros::topic& robot_ns,
                           const chsensors::config::proximity_sensor_config* const config)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.lidar"),
      ros_subscriber_sensor(robot_ns),
      m_use_exp(config->exp_decay) {
  enable();
  auto n_publishers = decoratee().getNumPublishers();

  ER_ASSERT(1 == n_publishers,
            "Expected 1 publisher of lidar data for %s, got %d",
            cpal::kRobotType.c_str(),
            n_publishers);
}

lidar_sensor::lidar_sensor(lidar_sensor&& other)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.lidar"),
      ros_subscriber_sensor(other.robot_ns()),
      m_use_exp(other.m_use_exp) {
  enable();
}

lidar_sensor& lidar_sensor::operator=(lidar_sensor&& rhs) {
  this->m_scan = rhs.m_scan;
  this->m_use_exp = rhs.m_use_exp;
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
  ER_ASSERT(is_enabled(), "%s called when disabled", __FUNCTION__);

  std::vector<rmath::vector2d> ret;
  size_t n_readings = 0;

  ER_DEBUG("Scan range_min=%f,range_max=%f",
           m_scan.range_min,
           m_scan.range_max);

  /*
   * Even though we get way more readings than from the IR sensors for the ARGoS
   * foot-bot, we don't want to downsample, because that will leave blindspots
   * which will make collision detection/obstacle avoidance much less reliable.
   */
  for (size_t i = 0; i < m_scan.ranges.size(); ++i) {
    auto angle = rmath::radians(m_scan.angle_min + i * m_scan.angle_increment);
    auto raw = m_scan.ranges[i];
    if (RCPPSW_IS_BETWEEN(m_scan.ranges[i],
                          m_scan.range_min,
                          m_scan.range_max)) {
      ++n_readings;


      double proc;
      if (m_use_exp) {
        proc = std::exp(-raw);
      }
      ER_TRACE("Processed reading %zu: angle=%f,raw=%f,proc=%f",
               i,
               angle.v(),
               raw,
               proc);
      ret.push_back(rmath::vector2d(proc, angle));
    } else {
      ER_TRACE("Discard reading %zu outside range: angle=%f,value=%f",
               i,
               angle.v(),
               raw);
    }
  } /* for(i..) */

  ER_DEBUG("Processed %zu valid readings", n_readings);
  return ret;
}

void lidar_sensor::callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  m_scan = *msg;
}

NS_END(sensors, ros, hal, cosm);
