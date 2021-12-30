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

#ifndef INCLUDE_COSM_HAL_ROS_ACTUATORS_ROS_ACTUATOR_HPP_
#define INCLUDE_COSM_HAL_ROS_ACTUATORS_ROS_ACTUATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include <ros/ros.h>

#include "cosm/hal/actuators/base_actuator.hpp"

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
class ros_actuator : public chactuators::base_actuator<::ros::Publisher> {
public:
  using impl_type = ::ros::Publisher;
  using chactuators::base_actuator<impl_type>::decoratee;

  /**
   * \brief If messages not consumed quickly enough, buffer up this many
   * messages before throwing some away.
   */
  static constexpr const size_t kQueueBufferSize = 1000;

  explicit ros_actuator(const impl_type& actuator)
      : chactuators::base_actuator<impl_type>(actuator) {}

  virtual ~ros_actuator(void) = default;

  ros_actuator(const ros_actuator&) = default;
  ros_actuator& operator=(const ros_actuator&) = default;
  ros_actuator(ros_actuator&&) = default;
  ros_actuator& operator=(ros_actuator&&) = default;

  /**
   * \brief To disable an actuator, you stop advertising the topic, which (I
   * think) only happens when your handle goes out of scope.
   */
  void disable(void) override {
    if (m_publishing) {
      decoratee().~impl_type();
      m_publishing = false;
    }
  }

  bool is_enabled(void) const override { return m_publishing; }

protected:
  template <typename TMsg>
      void advertise(const std::string& topic) {
    ::ros::NodeHandle nh;
    redecorate(nh.advertise<TMsg>(topic, kQueueBufferSize));
    m_publishing = true;
  }

private:
  /* clang-format off */
  bool m_publishing{true};
  /* clang-format on */
};

NS_END(actuators, ros, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ROS_ACTUATORS_ROS_ACTUATOR_HPP_ */
