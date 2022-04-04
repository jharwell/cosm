/**
 * \file ros_actuator.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/ros.h>

#include "cosm/hal/actuators/base_actuator.hpp"
#include "cosm/ros/topic.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, actuators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ros_actuator
 * \ingroup hal ros actuators
 *
 * \brief Base actuator class to provide a common interface to all ROS
 *        actuators.
 */
class ros_actuator : public rer::client<ros_actuator>,
                     public chactuators::base_actuator<::ros::Publisher> {
public:
  using impl_type = ::ros::Publisher;
  using chactuators::base_actuator<impl_type>::decoratee;

  /**
   * \brief If messages not consumed quickly enough, buffer up this many
   * messages before throwing some away.
   */
  static constexpr const size_t kQueueBufferSize = 1000;

  ros_actuator(const cros::topic& robot_ns);

  virtual ~ros_actuator(void) = default;

  ros_actuator(ros_actuator&&) = default;
  ros_actuator& operator=(ros_actuator&&) = default;
  ros_actuator(const ros_actuator&) = delete;
  ros_actuator& operator=(const ros_actuator&) = delete;

  /**
   * \brief To disable an actuator, you stop advertising the topic, which (I
   * think) only happens when your handle goes out of scope.
   */
  void disable(void) override;

  bool is_enabled(void) const override { return m_publishing; }

protected:
  template <typename TMsg>
      void advertise(const cros::topic& topic) {
    ::ros::NodeHandle nh;
    auto n_subs_old = decoratee().getNumSubscribers();
    redecorate(nh.advertise<TMsg>(topic, kQueueBufferSize));
    m_publishing = true;

    while (decoratee().getNumSubscribers() == n_subs_old) {
      ER_ASSERT(::ros::ok(),
                "Unable to wait for subscriber connection--ros::ok() failed");

      /* For real robots, things take a while to come up so we have to wait */
      ::ros::spinOnce();
      ::ros::Duration(1.0).sleep();

      ER_DEBUG("Wait for subscriber connection on topic '%s'",
               topic.c_str());
    }
    m_publishing = true;
    ER_INFO("Topic '%s' publishing active: %u subscribers (old=%u)",
            topic.c_str(),
            decoratee().getNumSubscribers(),
            n_subs_old);
  }
  cros::topic robot_ns(void) const { return m_robot_ns; }

private:
  /* clang-format off */
  cros::topic m_robot_ns;
  bool        m_publishing{false};
  /* clang-format on */
};

NS_END(actuators, ros, hal, cosm);
