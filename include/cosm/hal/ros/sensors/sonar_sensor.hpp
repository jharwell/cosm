/**
 * \file sonar_sensor.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::hal::ros::sensors {

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

} /* namespace cosm::hal::ros::sensors */
