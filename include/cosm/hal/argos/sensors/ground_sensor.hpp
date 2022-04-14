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
#include "cosm/hal/sensors/env_sensor_reading.hpp"

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

  explicit ground_sensor_impl(impl_type * const sensor)
      : ER_CLIENT_INIT("cosm.hal.argos.sensors.ground"),
        chargos::sensors::argos_sensor<impl_type>(sensor) {}
  ~ground_sensor_impl(void) override = default;

  /* move only constructible/assignable for use with saa subsystem */
  const ground_sensor_impl& operator=(const ground_sensor_impl&) = delete;
  ground_sensor_impl(const ground_sensor_impl&) = delete;
  ground_sensor_impl& operator=(ground_sensor_impl&&) = default;
  ground_sensor_impl(ground_sensor_impl&&) = default;

  /**
   * \brief Get the current ground sensor readings for the footbot robot.
   *
   * \return A vector of \ref reading.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_footbot_ground_sensor<U>::value)>
  std::vector<chsensors::env_sensor_reading> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    std::vector<chsensors::env_sensor_reading> ret;
    for (auto &r : decoratee()->GetReadings()) {
      ret.emplace_back(r.Value);
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
  std::vector<chsensors::env_sensor_reading> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    std::vector<chsensors::env_sensor_reading> ret;
    double tmp[3];
    decoratee()->GetReadings(tmp);
    ret.emplace_back(tmp[0]);
    ret.emplace_back(tmp[1]);
    ret.emplace_back(tmp[2]);
    return ret;
  }

  /**
   * \brief Get the current ground sensor readings for the pipuck robot.
   *
   * \return A vector of \ref reading.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_pipuck_ground_sensor<U>::value)>
  std::vector<chsensors::env_sensor_reading> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    chsensors::env_sensor_reading r;
    auto cb = [&](const auto& interface) {
                r.value = interface.Reflected;
              };
    decoratee()->Visit(cb);
    return {r};
  }
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
