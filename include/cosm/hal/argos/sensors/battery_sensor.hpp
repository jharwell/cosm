/**
 * \file battery_sensor.hpp
 *
 * \copyright 2018 The Nimer Inc., All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_ARGOS_SENSORS_BATTERY_SENSOR_HPP_
#define INCLUDE_COSM_HAL_ARGOS_SENSORS_BATTERY_SENSOR_HPP_

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>

#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/argos/sensors/argos_sensor.hpp"

/******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_BatterySensor;
} /* namespace argos */

NS_START(cosm, hal, argos, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_battery_sensor = std::is_same<TSensor, ::argos::CCI_BatterySensor>;

NS_END(detail);

/******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class battery_sensor_impl
 * \ingroup hal argos sensors
 *
 * \brief Battery sensor wrapper for the following supported robots:
 *
 * - ARGoS footbot
 * - ARGoS epuck
 * - ARGoS pipuck
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor.
 */
template <typename TSensor>
class battery_sensor_impl final : public rer::client<battery_sensor_impl<TSensor>>,
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
   * \brief A battery sensor reading.
   *
   * The first argument is the amount of battery left as a percent between 0 and
   * 1, and the second argument is the amount of time left for the robot.
   *
   * Note that these are different representations of the same essential
   * information: how much energy the robot current has left.
   */
  struct sensor_reading {
    double availible_charge;
    double time_left;
  };

  explicit battery_sensor_impl(impl_type * const sensor)
      : ER_CLIENT_INIT("cosm.hal.argos.sensors.battery"),
        chargos::sensors::argos_sensor<impl_type>(sensor) {}

  const battery_sensor_impl& operator=(const battery_sensor_impl&) = delete;
  battery_sensor_impl(const battery_sensor_impl&) = default;

  /**
   * \brief Get the current battery sensor reading for the footbot robot.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_battery_sensor<U>::value)>
  sensor_reading reading(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    auto temp = decoratee()->GetReading();
    return {temp.AvailableCharge, temp.TimeLeft};
  }
};

using battery_sensor = battery_sensor_impl<::argos::CCI_BatterySensor>;

NS_END(sensors, argos, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ARGOS_SENSORS_BATTERY_SENSOR_HPP_ */
