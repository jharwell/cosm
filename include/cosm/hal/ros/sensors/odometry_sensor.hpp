/**
 * \file odometry_sensor.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_ROS_SENSORS_ODOMETRY_SENSOR_HPP_
#define INCLUDE_COSM_HAL_ROS_SENSORS_ODOMETRY_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <limits>

#include <nav_msgs/Odometry.h>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/cosm.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/kin/odometry.hpp"
#include "cosm/hal/ros/sensors/ros_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class odometry_sensor_impl
 * \ingroup hal ros sensors
 *
 * \brief Odometry sensor.
 *
 * Supports the following robots:
 *
 * - ROS turtlebot3
 */
class odometry_sensor final : public rer::client<odometry_sensor>,
                              public chros::sensors::ros_sensor {
 public:
  odometry_sensor(void)
      : ER_CLIENT_INIT("cosm.hal.ros.sensors.odometry"),
        ros_sensor(::ros::NodeHandle().subscribe(kOdometryTopic,
                                                 kQueueBufferSize,
                                                 &odometry_sensor::callback,
                                                 this)) {}

  const odometry_sensor& operator=(const odometry_sensor&) = delete;
  odometry_sensor(const odometry_sensor&) = default;

  /**
   * \brief Get the current odometry sensor readings for the robot.
   *
   * \return A \ref ckin::odometry reading.
   */
  ckin::odometry reading(void) const {
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);
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
    ret.pose.orientation = rmath::euler_angles(rmath::radians(m_odom.pose.pose.orientation.x),
                                               rmath::radians(m_odom.pose.pose.orientation.y),
                                               rmath::radians(m_odom.pose.pose.orientation.z));

    return ret;
  }

  /**
   * \brief Reset the odometry; will be filled again the next time the publisher
   * publishes.
   */
  void reset(void) override { m_odom = {}; }

  void enable(void) override {
    subscribe(kOdometryTopic, &odometry_sensor::callback, this);
  }

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_TURTLEBOT3)
  void callback(const nav_msgs::Odometry::ConstPtr& msg) {
    m_odom = *msg;
  }
  static inline const std::string kOdometryTopic = "odom";
#endif /* COSM_HAL_TARGET */

  nav_msgs::Odometry m_odom{};
};

NS_END(sensors, ros, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ROS_SENSORS_ODOMETRY_SENSOR_HPP_ */
