/**
 * \file ground_sensor.hpp
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

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_ground_sensor.h>
#endif /* COSM_HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_FootBotMotorGroundSensor;
class CCI_EPuckGroundSensor;
} /* namespace argos */

NS_START(cosm, hal, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_argos_footbot_ground_sensor = std::is_same<TSensor,
                                                    argos::CCI_FootBotMotorGroundSensor>;

template<typename TSensor>
using is_argos_epuck_ground_sensor = std::is_same<TSensor,
                                                  argos::CCI_EPuckGroundSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ground_sensor_impl
 * \ingroup hal sensors
 *
 * \brief Ground sensor wrapper for the following supported robots:
 *
 * - ARGoS footbot
 * - ARGoS epuck
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor
 *                  at compile time, and SFINAE ensures no member functions can
 *                  be called.
 */
template <typename TSensor>
class ground_sensor_impl : public rer::client<ground_sensor_impl<TSensor>> {
 public:
  using impl_type = TSensor;

  static constexpr const char kNestTarget[] = "nest";

  /**
   * \brief A ground sensor reading (value, distance) pair.
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

  ground_sensor_impl(TSensor * const sensor,
                 const config::ground_sensor_config* const config)
      : ER_CLIENT_INIT("cosm.hal.sensors.ground_sensor"),
        mc_config(*config),
        m_sensor(sensor) {}
  ~ground_sensor_impl(void) override = default;

  const ground_sensor_impl& operator=(const ground_sensor_impl&) = delete;
  ground_sensor_impl(const ground_sensor_impl&) = default;

  /**
   * \brief Get the current ground sensor readings for the footbot robot.
   *
   * \return A vector of \ref reading.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_footbot_ground_sensor<U>::value)>
  std::vector<reading> readings(void) const {
    std::vector<reading> ret;
    for (auto &r : m_sensor->GetReadings()) {
      ret.emplace_back(r.Value, -1.0);
    } /* for(&r..) */

    return ret;
  }

  /**
   * \brief Get the current ground sensor readings for the epuck robot.
   *
   * \return A vector of \ref reading.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_epuck_ground_sensor<U>::value)>
  std::vector<reading> readings(void) const {
    std::vector<reading> ret;
    double tmp[3];
    m_sensor->GetReadings(tmp);
    ret.emplace_back(tmp[0], -1.0);
    ret.emplace_back(tmp[1], -1.0);
    ret.emplace_back(tmp[2], -1.0);
    return ret;
  }

  /**
   * \brief Detect if a certain condition is met by examining ground sensor
   * readings.
   *
   * \param name The name of the configured detection to check.
   *
   * \return \c TRUE iff the condition was detected by the specified # readings.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_footbot_ground_sensor<U>::value ||
                               detail::is_argos_epuck_ground_sensor<U>::value)>
  bool detect(const std::string& name) const {
    ER_ASSERT(mc_config.detect_map.end() != mc_config.detect_map.find(name),
              "Detection %s not found in configured map",
              name.c_str());
    auto &detection = mc_config.detect_map.find(name)->second;

    size_t sum = 0;
    for (auto &r : readings()) {
      sum += static_cast<size_t>(detection.range.contains(r.value));
    } /* for(&r..) */
    return  sum >= detection.consensus;
  }

 private:
  /* clang-format off */
  const config::ground_sensor_config mc_config;
  TSensor* const                     m_sensor;
  /* clang-format on */
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
using ground_sensor = ground_sensor_impl<argos::CCI_FootBotMotorGroundSensor>;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using ground_sensor = ground_sensor_impl<argos::CCI_EPuckGroundSensor>;
#else
class ground_sensor{};
#endif /* COSM_HAL_TARGET */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_GROUND_SENSOR_HPP_ */
