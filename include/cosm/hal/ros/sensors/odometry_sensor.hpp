/**
 * \file odometry_sensor.hpp
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

#include <nav_msgs/Odometry.h>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/cosm.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/kin/odometry.hpp"
#include "cosm/hal/ros/sensors/ros_subscriber_sensor.hpp"
#include "cosm/ros/topic.hpp"

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
 * - ROS extended turtlebot3
 */
class odometry_sensor final : public rer::client<odometry_sensor>,
                              public chros::sensors::ros_subscriber_sensor {
 public:
  explicit odometry_sensor(const cros::topic& robot_ns);

  /* copy constructible/assignable to work with the saa subsystem */
  odometry_sensor(odometry_sensor&&);
  odometry_sensor& operator=(odometry_sensor&&);

  /**
   * \brief Get the current odometry sensor readings for the robot.
   *
   * \return A \ref ckin::odometry reading.
   */
  ckin::odometry reading(void) const;

  /**
   * \brief Reset the odometry; will be filled again the next time the publisher
   * publishes.
   */
  void reset(void) override;

  void enable(void) override;

 private:
#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
  static inline const cros::topic kOdometryTopic = "odom";
#endif /* COSM_HAL_TARGET */

  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  /* clang-format off */
  nav_msgs::Odometry m_odom{};
  /* clang-format on */
};

NS_END(sensors, ros, hal, cosm);
