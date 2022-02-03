/**
 * \file colored_blob_camera_sensor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_ROS_SENSORS_COLORED_BLOB_CAMERA_SENSOR_HPP_
#define INCLUDE_COSM_HAL_ROS_SENSORS_COLORED_BLOB_CAMERA_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include <ros/ros.h>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/utils/color.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/cosm.hpp"
#include "cosm/hal/ros/sensors/ros_sensor.hpp"
#include "cosm/hal/sensors/colored_blob_camera_sensor_reading.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class colored_blob_camera_sensor
 * \ingroup hal ros sensors
 *
 * \brief Omnidirectional colored blob camera sensor wrapper (stub for now).
 *
 * Supports the following robots:
 *
 * - ROS turtlebot3
 */
class colored_blob_camera_sensor final : public rer::client<colored_blob_camera_sensor>,
                                         public chros::sensors::ros_sensor {
 public:
  using reading_type = chsensors::colored_blob_camera_sensor_reading;
  colored_blob_camera_sensor(const cros::topic& robot_ns)
      : ER_CLIENT_INIT("cosm.hal.sensors.colored_blob_camera"),
        ros_sensor(robot_ns) {
    disable();
  }

  const colored_blob_camera_sensor& operator=(const colored_blob_camera_sensor&) = delete;
  colored_blob_camera_sensor(const colored_blob_camera_sensor&) = delete;
  colored_blob_camera_sensor& operator=(colored_blob_camera_sensor&&) = default;
  colored_blob_camera_sensor(colored_blob_camera_sensor&&) = default;

  /**
   * \brief Get the sensor readings for the robot.
   *
   * \param ref The angle of the reference frame to use (i.e., the robot's
   *            current heading).
   *
   * \return A vector of \ref reading_type.
   */
  std::vector<reading_type>  readings(
      const rmath::radians& rframe = rmath::radians::kZERO) const {
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    std::vector<reading_type> ret;
    return ret;
  }

  void reset(void) override {
    /* TBD */
  }

  void enable(void) override {
    /* TBD */
  }
};

NS_END(sensors, ros, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ROS_SENSORS_COLORED_BLOB_CAMERA_SENSOR_HPP_ */
