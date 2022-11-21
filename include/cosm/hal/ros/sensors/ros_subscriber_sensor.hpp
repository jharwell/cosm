/**
 * \file ros_subscriber_sensor.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
namespace cosm::hal::ros::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ros_subscriber_sensor
 * \ingroup hal ros sensors
 *
 * \brief Base sensor class to provide a common interface to sensors which
 * use published ROS topics to function.
 */
class ros_subscriber_sensor : public rer::client<ros_subscriber_sensor>,
                   public chsensors::base_sensor<::ros::Subscriber> {
public:
  using impl_type = ::ros::Subscriber;
  using chsensors::base_sensor<impl_type>::decoratee;

  /**
   * \brief If messages are published more quickly than they can be sent, buffer
   * up this many messages before throwing some away.
   */
  static constexpr const size_t kQueueBufferSize = 1000;

  explicit ros_subscriber_sensor(const cros::topic& robot_ns);

  virtual ~ros_subscriber_sensor(void) = default;

  ros_subscriber_sensor(const ros_subscriber_sensor&) = delete;
  ros_subscriber_sensor& operator=(const ros_subscriber_sensor&) = delete;
  ros_subscriber_sensor(ros_subscriber_sensor&&) = default;
  ros_subscriber_sensor& operator=(ros_subscriber_sensor&&) = default;

  /**
   * \brief To disable a sensor, you unsubscribe from the topic, which (I think)
   * only happens when your handle goes out of scope.
   */
  void disable(void) override;

  bool is_enabled(void) const override { return m_topic != ""; }

protected:
  template <typename TCallback, typename TClass>
      void subscribe(const cros::topic& topic, TCallback cb, TClass* inst) {
    ::ros::NodeHandle nh;
    auto n_pubs_old = decoratee().getNumPublishers();
    m_topic = topic;
    redecorate(nh.subscribe(m_topic, kQueueBufferSize, cb, inst));

    while (decoratee().getNumPublishers() == n_pubs_old) {
      ER_ASSERT(::ros::ok(),
                "Unable to activate subscription--ros::ok() failed");

      /* For real robots, things take a while to come up so we have to wait */
      ::ros::spinOnce();
      ::ros::Duration(1.0).sleep();

      ER_DEBUG("Wait for topic '%s' subscription to activate",
               m_topic.c_str());
    }
    ER_INFO("Topic '%s' subscription active: %u publishers (old=%u)",
            m_topic.c_str(),
            decoratee().getNumPublishers(),
            n_pubs_old);
  }
  cros::topic robot_ns(void) const { return m_robot_ns; }

private:
  /* clang-format off */
  cros::topic m_robot_ns;
  cros::topic m_topic{};
  /* clang-format on */
};

} /* namespace cosm::hal::ros::sensors */
