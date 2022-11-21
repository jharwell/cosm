/**
 * \file light_sensor.hpp
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

#include <std_msgs/Float32.h>

#include "rcppsw/er/client.hpp"

#include "cosm/hal/ros/sensors/ros_service_sensor.hpp"
#include "cosm/hal/sensors/light_sensor_reading.hpp"
#include "cosm/hal/hal.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::ros::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class light_sensor
 * \ingroup hal ros sensors
 *
 * \brief Light sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ROS turtlebot3 (extended)
 */
class light_sensor : public rer::client<light_sensor>,
                     public chros::sensors::ros_service_sensor {
 public:
  using reading_type = chsensors::light_sensor_reading;

  explicit light_sensor(const cros::topic& robot_ns);

  /* move only constructible/assignable to work with the saa subsystem */
  light_sensor& operator=(const light_sensor&) = delete;
  light_sensor(const light_sensor&) = delete;
  light_sensor& operator=(light_sensor&& rhs);
  light_sensor(light_sensor&& other);

  void reset(void) override;
  void enable(void) override;

  /**
   * \brief Get the current light sensor readings for the footbot/epuck robots.
   *
   * \return A vector of \ref reading_type.
   */
  std::vector<reading_type> readings(void);

 private:
#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
  static inline const cros::topic kServiceName = "tsl2591/readings_service";
#endif

  /* clang-format off */
  std_msgs::Float32 m_light{};
  /* clang-format off */
};


} /* namespace cosm::hal::ros::sensors */
