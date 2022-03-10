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

#include <std_msgs/Float32.h>

#include "rcppsw/er/client.hpp"

#include "cosm/hal/ros/sensors/ros_sensor.hpp"

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
 * \brief Sonar sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ROS turtlebot3 (extended)
 */
class sonar_sensor : public rer::client<sonar_sensor>,
                     public chros::sensors::ros_sensor {
 public:
  struct reading_type {
    float value;
  };

  explicit sonar_sensor(const cros::topic& robot_ns);

  /* move only constructible/assignable to work with the saa subsystem */
  sonar_sensor& operator=(const sonar_sensor&) = delete;
  sonar_sensor(const sonar_sensor&) = delete;
  sonar_sensor& operator=(sonar_sensor&& rhs);
  sonar_sensor(sonar_sensor&& other);

  void reset(void) override;
  void enable(void) override;

  /**
   * \brief Get the current sonar sensor readings for the footbot/epuck robots.
   *
   * \return A vector of \ref reading.
   */
  std::vector<reading_type> readings(void) const;

 private:
#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
  static inline const cros::topic kSonarTopic = "sonar";
#endif /* COSM_HAL_TARGET */

  void callback(const std_msgs::Float32::ConstPtr& msg);

  /* clang-format off */
  std_msgs::Float32 m_sonar{};
  /* clang-format off */
};

NS_END(sensors, ros, hal, cosm);
