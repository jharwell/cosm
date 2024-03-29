/**
 * \file lidar_sensor.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include <sensor_msgs/LaserScan.h>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/ros/sensors/ros_subscriber_sensor.hpp"
#include "cosm/hal/sensors/config/proximity_sensor_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::ros::sensors {


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lidar_sensor
 * \ingroup hal ros sensors
 *
 * \brief Lidar sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ROS extended turtlebot3
 */
class lidar_sensor : public rer::client<lidar_sensor>,
                     public chros::sensors::ros_subscriber_sensor {
 public:
  lidar_sensor(const cros::topic& robot_ns,
               const chsensors::config::proximity_sensor_config* const config);

  /* move constructible/assignable to work with the saa subsystem */
  lidar_sensor(lidar_sensor&& other);
  lidar_sensor& operator=(lidar_sensor&& rhs);
  lidar_sensor(const lidar_sensor&) = delete;
  lidar_sensor& operator=(const lidar_sensor&) = delete;

  void reset(void) override;
  void enable(void) override;

  /**
   * \brief Get the current lidar sensor readings for the robot.
   *
   * \return A vector of (X,Y) pairs of sensor readings corresponding to
   * object distances.
   */
  std::vector<rmath::vector2d> readings(void) const;

 private:
#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
  static inline const cros::topic kScanTopic = "scan";
#endif /* COSM_HAL_TARGET */

  void callback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /* clang-format off */
  bool                   m_use_exp{true};
  sensor_msgs::LaserScan m_scan{};
  /* clang-format off */
};

} /* namespace cosm::hal::ros::sensors */
