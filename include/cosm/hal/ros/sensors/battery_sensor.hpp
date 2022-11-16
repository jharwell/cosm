/**
 * \file battery_sensor.hpp
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

#include <sensor_msgs/BatteryState.h>

#include "cosm/hal/sensors/metrics/battery_metrics.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/hal/ros/sensors/ros_subscriber_sensor.hpp"
#include "cosm/hal/sensors/battery_sensor_reading.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class battery_sensor
 * \ingroup hal ros sensors
 *
 * \brief Captures battery information from ROS and exposes it via traditional
 * interface.
 *
 * Supports the following robots:
 *
 * - ROS extended turtlebot3
 * - ROS turtlebot3
 */
class battery_sensor : public rer::client<battery_sensor>,
                       public chros::sensors::ros_subscriber_sensor,
                       public chsensors::metrics::battery_metrics {
 public:
  explicit battery_sensor(const cros::topic& robot_ns);

  /* move constructible/assignable to work with the saa subsystem */
  battery_sensor(battery_sensor&& other);
  battery_sensor& operator=(battery_sensor&& rhs);
  battery_sensor(const battery_sensor&) = delete;
  battery_sensor& operator=(const battery_sensor&) = delete;

  void reset(void) override;
  void enable(void) override;

  /* Battery metrics */
  double percent_remaining(void) const override;

  /**
   * \brief Get the current battery sensor readings for the robot.
   *
   * This returns a vector, not a scalar, so that future robots which have
   * multiple batteries will be able to use the same algorithms as this robot.
   */
  std::vector<chsensors::battery_sensor_reading> readings(void) const;

 private:
#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
  static inline const cros::topic kTopic = "battery_state";
#endif /* COSM_HAL_TARGET */

  void callback(const sensor_msgs::BatteryState::ConstPtr& msg);

  /* clang-format off */
  sensor_msgs::BatteryState m_state{};
  /* clang-format off */
};

NS_END(sensors, ros, hal, cosm);
