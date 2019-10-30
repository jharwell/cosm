/**
 * @file ground_sensor.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_SENSORS_GROUND_SENSOR_HPP_
#define INCLUDE_COSM_HAL_SENSORS_GROUND_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <string>

#include "rcppsw/rcppsw.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/sensors/config/ground_sensor_config.hpp"

#if HAL_CONFIG == HAL_CONFIG_ARGOS_FOOTBOT
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#else
#error "Selected hardware has no ground sensor!"
#endif /* HAL_CONFIG */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_argos_ground_sensor = std::is_same<TSensor,
                                            argos::CCI_FootBotMotorGroundSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class ground_sensor
 * @ingroup cosm hal sensors
 *
 * @brief Ground sensor wrapper for the following supported robots:
 *
 * - ARGoS footbot
 */
template <typename TSensor>
class _ground_sensor : public rer::client<_ground_sensor<TSensor>> {
 public:
  static constexpr char kNestTarget[] = "nest";

  /**
   * @brief A ground sensor reading (value, distance) pair.
   *
   * The first argument is the value of the sensor (a robot can have a variable
   * number of sensors), and the second
   * argument is the distance from the sensor on the robot to the robot's center
   * (this may not be used, depending on the actual hardware mapped to).
   */
  struct reading {
    reading(double v, double d) noexcept : value(v), distance(d) {}

    double value;
    double distance;
  };

  _ground_sensor(TSensor * const sensor,
                 const config::ground_sensor_config* const config)
      : ER_CLIENT_INIT("cosm.hal.sensors.ground_sensor"),
        mc_config(*config),
        m_sensor(sensor) {}
  ~_ground_sensor(void) override final = default;

  const _ground_sensor& operator=(const _ground_sensor&) = delete;
  _ground_sensor(const _ground_sensor&) = default;

  /**
   * @brief Get the current ground sensor readings for the footbot robot.
   *
   * @return A vector of \ref reading.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_ground_sensor<U>::value)>
  std::vector<reading> readings(void) const {
    std::vector<reading> ret;
    for (auto &r : m_sensor->GetReadings()) {
      ret.emplace_back(r.Value, -1.0);
    } /* for(&r..) */

    return ret;
  }

  /**
   * @brief Detect if a certain condition is met by examining footbot ground
   * sensor readings.
   *
   * @param name The name of the configured detection to check.
   *
   * @return \c TRUE iff the condition was detected by the specified # readings.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_ground_sensor<U>::value)>
  bool detect(const std::string& name) const {
    ER_ASSERT(mc_config.detect_map.end() != mc_config.detect_map.find(name),
              "Detection %s not found in configured map",
              name.c_str());
    auto &detection = mc_config.detect_map.find(name)->second;

    uint sum = 0;
    for (auto &r : readings()) {
      sum += static_cast<uint>(detection.range.contains(r.value));
    } /* for(&r..) */
    return  sum >= detection.consensus;
  }

 private:
  /* clang-format off */
  const config::ground_sensor_config mc_config;
  TSensor* const                     m_sensor;
  /* clang-format on */
};

#if HAL_CONFIG == HAL_CONFIG_ARGOS_FOOTBOT
using ground_sensor = _ground_sensor<argos::CCI_FootBotMotorGroundSensor>;
#endif /* HAL_CONFIG */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_GROUND_SENSOR_HPP_ */
