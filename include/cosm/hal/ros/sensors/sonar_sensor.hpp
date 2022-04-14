/**
 * \file sonar_sensor.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <string>

#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/ros/sensors/ros_service_sensor.hpp"
#include "cosm/hal/ros/sensors/config/sonar_sensor_config.hpp"
#include "cosm/hal/sensors/env_sensor_reading.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sonar_sensor
 * \ingroup hal ros sensors
 *
 * \brief Ultrasonic sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ROS turtlebot3 (extended)
 */
class sonar_sensor : public rer::client<sonar_sensor>,
                      public chros::sensors::ros_service_sensor {
 public:
  sonar_sensor(const cros::topic& robot_ns,
               const config::sonar_sensor_config* config);

  /* move only constructible/assignable to work with the saa subsystem */
  sonar_sensor& operator=(const sonar_sensor&) = delete;
  sonar_sensor(const sonar_sensor&) = delete;
  sonar_sensor& operator=(sonar_sensor&& rhs);
  sonar_sensor(sonar_sensor&& other);

  void reset(void) override {}
  void enable(void) override;

  /**
   * \brief Get the current sonar sensor readings.
   *
   * \return A vector of \ref reading.
   */
  std::vector<chsensors::env_sensor_reading> readings(void);

 private:
  /* clang-format off */
  chros::sensors::config::sonar_sensor_config m_config;
  /* clang-format on */
};

NS_END(sensors, ros, hal, cosm);
