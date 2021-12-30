/**
 * \file diff_drive_actuator.hpp
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

#ifndef INCLUDE_COSM_HAL_ROS_ACTUATORS_DIFF_DRIVE_ACTUATOR_HPP_
#define INCLUDE_COSM_HAL_ROS_ACTUATORS_DIFF_DRIVE_ACTUATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "rcppsw/er/client.hpp"

#include "cosm/hal/ros/actuators/ros_actuator.hpp"
#include "cosm/kin/twist.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, actuators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class diff_drive_actuator
 * \ingroup hal actuators
 *
 * \brief Differential drive actuator wrapper.
 *
 * Supports the following robots:
 *
 * - ROS turtlebot3
 */
class diff_drive_actuator : public rer::client<diff_drive_actuator>,
                            public chros::actuators::ros_actuator {
 public:
  /**
   * \brief Construct the wrapper actuator.
   *
   * \param handle Handle to the ROS node.
   */
  diff_drive_actuator(void)
      : ER_CLIENT_INIT("cosm.hal.ros.actuators.diff_drive"),
        chros::actuators::ros_actuator(
            ::ros::NodeHandle().advertise<geometry_msgs::Twist>(kCmdVelTopic,
                                                                kQueueBufferSize)) {}

  const diff_drive_actuator& operator=(const diff_drive_actuator&) = delete;
  diff_drive_actuator(const diff_drive_actuator&) = default;

  /**
   * \brief Stop the wheels of a robot. As far as I know, this is an immediate
   * stop (i.e. no rampdown).
   */
  void reset(void) override { set_from_twist({}, {}); }

  void enable(void) override {
    advertise<geometry_msgs::Twist>(kCmdVelTopic);
  }

  void set_from_twist(const ckin::twist& desired,
                      const rmath::range<rmath::radians>&) {
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    geometry_msgs::Twist t;
    t.linear.x = desired.linear.x();
    t.angular.z = desired.angular.z();

    decoratee().publish(t);
  }

 private:
#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_TURTLEBOT3)
  static inline const std::string kCmdVelTopic = "cmd_vel";
#endif /* COSM_HAL_TARGET */
};

NS_END(actuators, ros, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ROS_ACTUATORS_DIFF_DRIVE_ACTUATOR_HPP_ */
