/**
 * \file diff_drive_sensor.hpp
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

#ifndef INCLUDE_COSM_HAL_SENSORS_DIFF_DRIVE_SENSOR_HPP_
#define INCLUDE_COSM_HAL_SENSORS_DIFF_DRIVE_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"
#include "cosm/hal/hal.hpp"

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#else
#error "Selected hardware has no differential drive sensor!"
#endif /* HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename T>
using is_argos_ds_sensor = std::is_same<T,
                                          argos::CCI_DifferentialSteeringSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class diff_drive_sensor_impl
 * \ingroup hal sensors
 *
 * \brief Differential drive sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ARGoS footbot
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor
 *                  at compile time, and SFINAE ensures no member functions can
 *                  be called.
 */
template <typename TSensor>
class diff_drive_sensor_impl {
 public:
  struct reading {
    double vel_left;
    double vel_right;
    double dist_left;
    double dist_right;
    double axle_length;
  };

  explicit diff_drive_sensor_impl(TSensor* const sensor) : m_sensor(sensor) {}

  /**
   * \brief Get the current battery sensor reading for the footbot robot.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_ds_sensor<U>::value)>
  struct reading readings(void) const {
    auto tmp = m_sensor->GetReading();
    return {tmp.VelocityLeftWheel,
          tmp.VelocityRightWheel,
          tmp.CoveredDistanceLeftWheel,
          tmp.CoveredDistanceRightWheel,
          tmp.WheelAxisLength};
  }

  /**
   * \brief Return the current speed of the robot (average of the 2 wheel
   * speeds).
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_ds_sensor<U>::value)>
  double current_speed(void) const {
    auto tmp = readings();
    return (tmp.vel_left + tmp.vel_right) / 2.0;
  }

  void reset(void) {}

 private:
  /* clang-format off */
  TSensor* const m_sensor;
  /* clang-format on */
};

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
using diff_drive_sensor = diff_drive_sensor_impl<argos::CCI_DifferentialSteeringSensor>;
#endif /* HAL_TARGET */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORSDIFF_DRIVE_SENSOR_HPP_ */
