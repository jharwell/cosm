/**
 * \file ros_sensor.hpp
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

#ifndef INCLUDE_COSM_HAL_ROS_SENSORS_ROS_SENSOR_HPP_
#define INCLUDE_COSM_HAL_ROS_SENSORS_ROS_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include <ros/ros.h>

#include "cosm/hal/sensors/base_sensor.hpp"
#include "cosm/ros/topic.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ros_sensor
 * \ingroup hal ros sensors
 *
 * \brief Base sensor class to provide a common interface to all ROS sensors.
 */
class ros_sensor : public rer::client<ros_sensor>,
                   public chsensors::base_sensor<::ros::Subscriber> {
public:
  using impl_type = ::ros::Subscriber;
  using chsensors::base_sensor<impl_type>::decoratee;

  /**
   * \brief If messages are published more quickly than they can be sent, buffer
   * up this many messages before throwing some away.
   */
  static constexpr const size_t kQueueBufferSize = 1000;

  explicit ros_sensor(const cros::topic& robot_ns);

  virtual ~ros_sensor(void) = default;

  ros_sensor(const ros_sensor&) = delete;
  ros_sensor& operator=(const ros_sensor&) = delete;
  ros_sensor(ros_sensor&&) = default;
  ros_sensor& operator=(ros_sensor&&) = default;

  /**
   * \brief To disable a sensor, you unsubscribe from the topic, which (I think)
   * only happens when your handle goes out of scope.
   */
  void disable(void) override;

  bool is_enabled(void) const override { return m_subscribed; }

protected:
  template <typename TCallback, typename TClass>
      void subscribe(const cros::topic& topic, TCallback cb, TClass* inst) {
    ::ros::NodeHandle nh;
    auto n_pubs_old = decoratee().getNumPublishers();
    redecorate(nh.subscribe(topic, kQueueBufferSize, cb, inst));

    while(::ros::ok() && decoratee().getNumPublishers() == n_pubs_old) {
      ::ros::Duration(0.2).sleep();
      ER_DEBUG("Wait for topic '%s' subscription to activate",
               topic.c_str());
    }
    m_subscribed = true;
    ER_INFO("Topic '%s' subscription active",
             topic.c_str());
  }
  cros::topic robot_ns(void) const { return m_robot_ns; }


private:
  /* clang-format off */
  cros::topic m_robot_ns;
  bool        m_subscribed{false};
  /* clang-format on */
};

NS_END(sensors, ros, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ROS_SENSORS_ROS_SENSOR_HPP_ */
