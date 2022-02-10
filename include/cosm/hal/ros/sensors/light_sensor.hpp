/**
 * \file light_sensor.hpp
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
#include <string>

#include "rcppsw/er/client.hpp"

#include "cosm/hal/ros/sensors/ros_sensor.hpp"
#include "cosm/hal/sensors/light_sensor_reading.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class light_sensor
 * \ingroup hal ros sensors
 *
 * \brief Light sensor wrapper (stub for the moment).
 *
 * Supports the following robots:
 *
 * - ROS turtlebot3
 */
class light_sensor : public rer::client<light_sensor>,
                     public chros::sensors::ros_sensor {
 public:
  using reading_type = chal::sensors::light_sensor_reading;

  light_sensor(const cros::topic& robot_ns)
      : ER_CLIENT_INIT("cosm.hal.ros.sensors.light"),
        ros_sensor(robot_ns) {
    disable();
  }

  /* move only constructible/assignable to work with the saa subsystem */
  light_sensor& operator=(const light_sensor&) = delete;
  light_sensor(const light_sensor&) = delete;
  light_sensor& operator=(light_sensor&&) = default;
  light_sensor(light_sensor&&) = default;

  /**
   * \brief Get the current light sensor readings for the footbot/epuck robots.
   *
   * \return A vector of \ref reading.
   */
  std::vector<reading_type>  readings(void) const {
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    std::vector<reading_type> ret;
    return ret;
  }

  /**
   * \brief Detect if a certain condition is met by examining light sensor
   * readings.
   *
   * \param name The name of the configured detection to check.
   *
   * \return \c TRUE iff the condition was detected.
   */
  bool detect(const std::string& name) const {
    return false;
  }

  void reset(void) override {
    /* TBD */
  }

  void enable(void) override {
    /* TBD */
  }
};

NS_END(sensors, ros, hal, cosm);

