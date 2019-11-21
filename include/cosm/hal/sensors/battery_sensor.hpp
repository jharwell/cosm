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

#ifndef INCLUDE_COSM_HAL_SENSORS_BATTERY_SENSOR_HPP_
#define INCLUDE_COSM_HAL_SENSORS_BATTERY_SENSOR_HPP_

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"
#include "cosm/hal/hal.hpp"

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
#else
#error "Selected hardware has no battery sensor!"
#endif /* HAL_TARGET */

/******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_argos_battery_sensor = std::is_same<TSensor,
                                             argos::CCI_BatterySensor>;

NS_END(detail);

/******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class battery_sensor_impl
 * \ingroup hal sensors
 *
 * \brief Battery sensor wrapper for the following supported robots:
 *
 * - ARGoS footbot
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor
 *                  at compile time, and SFINAE ensures no member functions can
 *                  be called.
 */
template <typename TSensor>
class battery_sensor_impl {
 public:
  /**
   * \brief A battery sensor reading.
   *
   * The first argument is the amount of battery left as a percent between 0 and
   * 1, and the second argument is the amount of time left for the robot.
   *
   * Note that these are different representations of the same essential
   * information: how much energy the robot current has left.
   */
  struct reading {
    double availible_charge;
    double time_left;

    reading(double _availible_charge, double _time_left)
        : availible_charge(_availible_charge),
          time_left(_time_left) {}
  };

  explicit battery_sensor_impl(TSensor * const sensor) : m_sensor(sensor) {}

  /**
   * \brief Get the current battery sensor reading for the footbot robot.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_battery_sensor<U>::value)>
  struct reading readings(void) const {
    argos::CCI_BatterySensor::SReading temp = m_sensor->GetReading();
    reading ret(temp.AvailableCharge, temp.TimeLeft);
    return ret;
  }

 private:
  /* clang-format off */
  TSensor* const m_sensor;
  /* clang-format on */
};

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
using battery_sensor = battery_sensor_impl<argos::CCI_BatterySensor>;
#endif /* HAL_TARGET */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_BATTERY_SENSOR_HPP_ */
