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

#ifndef INCLUDE_COSM_HAL_ROS_SENSORS_LIDAR_SENSOR_HPP_
#define INCLUDE_COSM_HAL_ROS_SENSORS_LIDAR_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <limits>
#include <boost/optional.hpp>

#include "rcppsw/math/range.hpp"
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/ros/sensors/ros_sensor.hpp"

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
 * \brief Proxmity sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ROS turtlebot3 (stub for now)
 */
class lidar_sensor : public rer::client<lidar_sensor>,
                     public chros::sensors::ros_sensor {
 public:
lidar_sensor(void)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.lidar"),
      chros::sensors::ros_sensor({}) {}

  const lidar_sensor& operator=(const lidar_sensor&) = delete;
  lidar_sensor(const lidar_sensor&) = default;

  void reset(void) override { }

  void enable(void) override { }

  /**
   * \brief Get the current lidar sensor readings for the robot.
   *
   * \return A vector of (X,Y) pairs of sensor readings corresponding to
   * object distances.
   */
  std::vector<rmath::vector2d> readings(void) const {
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    std::vector<rmath::vector2d> ret;

    return ret;
  }
};

NS_END(sensors, ros, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ROS_SENSORS_LIDAR_SENSOR_HPP_ */
