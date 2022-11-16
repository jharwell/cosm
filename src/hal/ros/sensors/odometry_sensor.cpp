/**
 * \file odometry_sensor.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/ros/sensors/odometry_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
odometry_sensor::odometry_sensor(const cros::topic& robot_ns)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.odometry"),
      ros_subscriber_sensor(robot_ns) {
  enable();
  auto n_publishers = decoratee().getNumPublishers();
  ER_ASSERT(1 == n_publishers,
            "Expected 1 publisher of odometry for %s, got %d",
            cpal::kRobotType.c_str(),
            n_publishers);
}

odometry_sensor::odometry_sensor(odometry_sensor&& other)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.odometry_sensor"),
      ros_subscriber_sensor(other.robot_ns()) {
  enable();
}

odometry_sensor& odometry_sensor::operator=(odometry_sensor&& rhs) {
  this->m_odom = rhs.m_odom;
  rhs.disable();
  this->enable();
  return *this;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ckin::odometry odometry_sensor::reading(void) const {
  ER_ASSERT(is_enabled(), "%s called when disabled", __FUNCTION__);
  ckin::odometry ret;
  ret.twist.linear = rmath::vector3d(m_odom.twist.twist.linear.x,
                                     m_odom.twist.twist.linear.y,
                                     m_odom.twist.twist.linear.z);

  /*
     * If speed comes back as 0.0, then we are executing a hard turn, probably
     * as we vector somewhere. In order to have the arrival force work properly,
     * we need to have a velocity with a non-zero length and the correct heading
     * angle at all times. So we report that we have velocity even though we do
     * not, for the purposes of making those calculations work.
     *
     * There probably is a better way to do this, but I don't know what it is.
     */
  if (ret.twist.linear.length() <= std::numeric_limits<double>::epsilon()) {
    ret.twist.linear = rmath::vector3d::X * 0.01;
  }

  ret.twist.angular = rmath::vector3d(m_odom.twist.twist.angular.x,
                                      m_odom.twist.twist.angular.y,
                                      m_odom.twist.twist.angular.z);
  ret.pose.position = rmath::vector3d(m_odom.pose.pose.position.x,
                                      m_odom.pose.pose.position.y,
                                      m_odom.pose.pose.position.z);
  ret.pose.orientation =
      rmath::euler_angles(rmath::radians(m_odom.pose.pose.orientation.x),
                          rmath::radians(m_odom.pose.pose.orientation.y),
                          rmath::radians(m_odom.pose.pose.orientation.z));

  return ret;
}

void odometry_sensor::reset(void) { m_odom = {}; }

void odometry_sensor::enable(void) {
  if (is_enabled()) {
    return;
  }
  auto topic = robot_ns() / cros::topic(kOdometryTopic);
  ER_INFO("%s: ns=%s, topic=%s, subscribe=%s",
          __FUNCTION__,
          robot_ns().c_str(),
          kOdometryTopic.c_str(),
          topic.c_str());

  subscribe(topic, &odometry_sensor::callback, this);
}

void odometry_sensor::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  m_odom = *msg;
}

NS_END(sensors, ros, hal, cosm);
