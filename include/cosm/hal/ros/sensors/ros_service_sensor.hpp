/**
 * \file ros_service_sensor.hpp
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
 * \class ros_service_sensor
 * \ingroup hal ros sensors
 *
 * \brief Base sensor class to provide a common interface to sensors which use
 * ROS services to function.
 */
class ros_service_sensor : public rer::client<ros_service_sensor>,
                           public chsensors::base_sensor<::ros::ServiceClient> {
public:
  using impl_type = ::ros::ServiceClient;
  using chsensors::base_sensor<impl_type>::decoratee;

  explicit ros_service_sensor(const cros::topic& robot_ns);

  virtual ~ros_service_sensor(void) = default;

  ros_service_sensor(const ros_service_sensor&) = delete;
  ros_service_sensor& operator=(const ros_service_sensor&) = delete;
  ros_service_sensor(ros_service_sensor&&) = default;
  ros_service_sensor& operator=(ros_service_sensor&&) = default;

  /**
   * \brief To disable a sensor, you unsubscribe from the topic, which (I think)
   * only happens when your handle goes out of scope.
   */
  void disable(void) override;

  bool is_enabled(void) const override { return m_name != ""; }

protected:
  template <typename TServiceData>
      void connect(const cros::topic& name) {
    ::ros::NodeHandle nh;
    m_name = name;
    redecorate(nh.serviceClient<TServiceData>(m_name));

    while (!decoratee().waitForExistence(::ros::Duration(1.0))) {
      ER_ASSERT(::ros::ok(),
                "Unable to connect to service--ros::ok() failed");

      /* For real robots, things take a while to come up so we have to wait */
      ::ros::spinOnce();

      ER_DEBUG("Wait for service '%s' to become available",
               m_name.c_str());
    }
    ER_INFO("Connected to service '%s'", m_name.c_str())
  }
  cros::topic robot_ns(void) const { return m_robot_ns; }
  cros::topic service_name(void) const { return m_name; }

private:
  /* clang-format off */
  cros::topic m_robot_ns;
  cros::topic m_name{};
  /* clang-format on */
};

} /* namespace cosm::hal::ros::sensors */
