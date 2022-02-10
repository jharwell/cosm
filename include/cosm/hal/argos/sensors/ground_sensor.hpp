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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <string>

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_ground_sensor.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_ground_sensor.h>
#endif /* COSM_HAL_TARGET */

#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/argos/sensors/argos_sensor.hpp"
#include "cosm/hal/argos/sensors/config/ground_sensor_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_FootBotMotorGroundSensor;
class CCI_EPuckGroundSensor;
class CCI_PiPuckGroundSensor;
} /* namespace argos */

NS_START(cosm, hal, argos, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_footbot_ground_sensor = std::is_same<TSensor,
                                              ::argos::CCI_FootBotMotorGroundSensor>;

template<typename TSensor>
using is_epuck_ground_sensor = std::is_same<TSensor,
                                            ::argos::CCI_EPuckGroundSensor>;

template<typename TSensor>
using is_pipuck_ground_sensor = std::is_same<TSensor,
                                             ::argos::CCI_PiPuckGroundSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ground_sensor_impl
 * \ingroup hal argos sensors
 *
 * \brief Ground sensor wrapper for the following supported robots:
 *
 * - ARGoS footbot
 * - ARGoS epuck
 * - ARGoS pipuck
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor.
 */
template <typename TSensor>
class ground_sensor_impl : public rer::client<ground_sensor_impl<TSensor>>,
                           public chargos::sensors::argos_sensor<TSensor> {
 private:
  using chargos::sensors::argos_sensor<TSensor>::decoratee;

 public:
  using impl_type = TSensor;
  using chargos::sensors::argos_sensor<impl_type>::enable;
  using chargos::sensors::argos_sensor<impl_type>::disable;
  using chargos::sensors::argos_sensor<impl_type>::reset;
  using chargos::sensors::argos_sensor<impl_type>::is_enabled;

  /**
   * \brief A ground sensor reading (value, distance) pair.
   *
   * The first argument is the value of the sensor (a robot can have a variable
   * number of sensors), and the second
   * argument is the distance from the sensor on the robot to the robot's center
   * (this may not be used, depending on the actual hardware mapped to).
   */
  struct reading {
    reading(void) = default;
    reading(double v, double d) noexcept : value(v), distance(d) {}

    double value{-1.0};
    double distance{-1.0};
  };

  ground_sensor_impl(impl_type * const sensor,
                     const chargos::sensors::config::ground_sensor_config* const config)
      : ER_CLIENT_INIT("cosm.hal.argos.sensors.ground"),
        chargos::sensors::argos_sensor<impl_type>(sensor),
        m_config(*config) {}
  ~ground_sensor_impl(void) override = default;

  /* move only constructible/assignable for use with saa subsystem */
  const ground_sensor_impl& operator=(const ground_sensor_impl&) = delete;
  ground_sensor_impl(const ground_sensor_impl&) = delete;
  ground_sensor_impl& operator=(ground_sensor_impl&&) = default;
  ground_sensor_impl(ground_sensor_impl&&) = default;

  void config_update(
      const chargos::sensors::config::ground_sensor_config* const config) {
    m_config = *config;
  }
  /**
   * \brief Get the current ground sensor readings for the footbot robot.
   *
   * \return A vector of \ref reading.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_footbot_ground_sensor<U>::value)>
  std::vector<reading> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    std::vector<reading> ret;
    for (auto &r : decoratee()->GetReadings()) {
      ret.emplace_back(r.Value, -1.0);
    } /* for(&r..) */

    return ret;
  }

  /**
   * \brief Get the current ground sensor readings for the epuck robot.
   *
   * \return A vector of \ref reading.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_epuck_ground_sensor<U>::value)>
  std::vector<reading> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    std::vector<reading> ret;
    double tmp[3];
    decoratee()->GetReadings(tmp);
    ret.emplace_back(tmp[0], -1.0);
    ret.emplace_back(tmp[1], -1.0);
    ret.emplace_back(tmp[2], -1.0);
    return ret;
  }

  /**
   * \brief Get the current ground sensor readings for the pipuck robot.
   *
   * \return A vector of \ref reading.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_pipuck_ground_sensor<U>::value)>
  std::vector<reading> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    reading r;
    auto cb = [&](const auto& interface) {
                r.value = interface.Reflected;
                r.distance = -1.0;
              };
    decoratee()->Visit(cb);
    return {r};
  }

  /**
   * \brief Detect if a certain condition is met by examining ground sensor
   * readings.
   *
   * \param name The name of the configured detection to check.
   *
   * \return \c TRUE iff the condition was detected.
   */
    bool detect(const std::string& name) const {
    ER_ASSERT(m_config.detect_map.end() != m_config.detect_map.find(name),
              "Detection %s not found in configured map",
              name.c_str());
    auto &detection = m_config.detect_map.find(name)->second;

    size_t sum = 0;
    for (auto &r : readings()) {
      sum += static_cast<size_t>(detection.range.contains(r.value));
    } /* for(&r..) */
    return  sum >= detection.consensus;
  }
 private:
  /* clang-format off */
  chargos::sensors::config::ground_sensor_config m_config;
  /* clang-format on */
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
using ground_sensor = ground_sensor_impl<::argos::CCI_FootBotMotorGroundSensor>;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using ground_sensor = ground_sensor_impl<::argos::CCI_EPuckGroundSensor>;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
using ground_sensor = ground_sensor_impl<::argos::CCI_PiPuckGroundSensor>;
#else
class ground_sensor{};
#endif /* COSM_HAL_TARGET */

NS_END(sensors, argos, hal, cosm);

