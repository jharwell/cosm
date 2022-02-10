/**
 * \file diff_drive_actuator.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/ros/actuators/diff_drive_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, actuators);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
diff_drive_actuator::diff_drive_actuator(const cros::topic& robot_ns)
    : ER_CLIENT_INIT("cosm.hal.ros.actuators.diff_drive"),
      ros_actuator(robot_ns) {
  enable();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void diff_drive_actuator::reset(void) { set_from_twist({}, {}); }

void diff_drive_actuator::enable(void) {
  if (is_enabled()) {
    return;
  }
  auto topic = robot_ns() / cros::topic(kCmdVelTopic);
  ER_INFO("%s: ns=%s, topic=%s, advertise=%s",
          __FUNCTION__,
          robot_ns().c_str(),
          kCmdVelTopic.c_str(),
          topic.c_str());

  advertise<geometry_msgs::Twist>(topic);
}

void diff_drive_actuator::set_from_twist(const ckin::twist& desired,
                                         const rmath::range<rmath::radians>& soft_turn,
                                         double) {
  ER_ASSERT(is_enabled(),
            "%s called when disabled",
            __FUNCTION__);

  geometry_msgs::Twist t;
  auto angle = rmath::radians(desired.angular.z()).signed_normalize();
  t.linear.x = desired.linear.x();
  t.angular.z = angle.v();

  ER_DEBUG("Linear_x=%f, angular_z=%f, soft_turn=%s",
           t.linear.x,
           angle.v(),
           rcppsw::to_string(soft_turn).c_str());
  decoratee().publish(t);
}



NS_END(actuators, ros, hal, cosm);
