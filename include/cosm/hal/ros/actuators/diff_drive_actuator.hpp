/**
 * \file diff_drive_actuator.hpp
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
#include <geometry_msgs/Twist.h>

#include "rcppsw/er/client.hpp"

#include "cosm/hal/ros/actuators/ros_actuator.hpp"
#include "cosm/kin/twist.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/hal/actuators/locomotion_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::ros::actuators {

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
                            public chros::actuators::ros_actuator,
                            public chactuators::locomotion_actuator {
 public:
  /**
   * \brief Construct the wrapper actuator.
   */
  explicit diff_drive_actuator(const cros::topic& robot_ns);

  /* move constructible/assignable to work with the saa subsystem */
  diff_drive_actuator(diff_drive_actuator&&) = default;
  diff_drive_actuator& operator=(diff_drive_actuator&&) = default;
  diff_drive_actuator(const diff_drive_actuator&) = delete;
  diff_drive_actuator& operator=(const diff_drive_actuator&) = delete;

  /* locomotion actuator overrides */
  void stop(void) override { reset(); }
  double max_velocity(void) const override { return mc_max_velocity; }

  /**
   * \brief Stop the wheels of a robot. As far as I know, this is an immediate
   * stop (i.e. no rampdown).
   */
  void reset(void) override;

  void enable(void) override;

  void set_from_twist(const ckin::twist& desired,
                      const rmath::range<rmath::radians>& soft_turn);

 private:
#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
  static inline const std::string kCmdVelTopic = "cmd_vel";

  /*
   * From https://emanual.robotis.com/docs/en/platform/turtlebot3/features/, burger
   */
  const double mc_max_velocity{0.22}; /* m/s */
#endif /* COSM_HAL_TARGET */


};

} /* namespace cosm::hal::ros::actuators */
