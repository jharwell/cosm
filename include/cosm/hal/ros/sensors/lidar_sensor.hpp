/**
 * \file lidar_sensor.hpp
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
#include <vector>

#include <sensor_msgs/LaserScan.h>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/ros/sensors/ros_subscriber_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);


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
  explicit lidar_sensor(const cros::topic& robot_ns);

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
  sensor_msgs::LaserScan m_scan{};
  /* clang-format off */
};

NS_END(sensors, ros, hal, cosm);
