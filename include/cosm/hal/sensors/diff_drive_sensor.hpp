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

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#endif /* COSM_HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_DifferentialSteeringSensor;
class CCI_PiPuckDifferentialDriveSensor;
} /* namespace argos */

NS_START(cosm, hal, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename T>
using is_argos_generic_ds_sensor = std::is_same<T,
                                                argos::CCI_DifferentialSteeringSensor>;
template<typename T>
using is_argos_pipuck_ds_sensor = std::is_same<T,
                                               argos::CCI_PiPuckDifferentialDriveSensor>;

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
 * - ARGoS epuck
 * - ARGoS pipuck
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                 HAL. If nullptr, then that effectively disables the sensor
 *                 at compile time, and SFINAE ensures no member functions can
 *                 be called.
 */
template <typename TSensor>
class diff_drive_sensor_impl {
 public:
  using impl_type = TSensor;

  struct sensor_reading {
    double vel_left;
    double vel_right;
    double dist_left;
    double dist_right;
    double axle_length;
  };

  explicit diff_drive_sensor_impl(TSensor* const sensor) : m_sensor(sensor) {}

  /**
   * \brief Get the current differential drive reading for the robot.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_generic_ds_sensor<U>::value)>
  sensor_reading reading(void) const {
    auto tmp = m_sensor->GetReading();
    return {tmp.VelocityLeftWheel,
          tmp.VelocityRightWheel,
          tmp.CoveredDistanceLeftWheel,
          tmp.CoveredDistanceRightWheel,
          tmp.WheelAxisLength};
  }

  /**
   * \brief Get the current differential drive reading for the robot.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_pipuck_ds_sensor<U>::value)>
  sensor_reading reading(void) const {
    return {m_sensor->GetLeftVelocity(),
            m_sensor->GetRightVelocity(),
            0.0,
            0.0,
            0.0};
  }

  /**
   * \brief Return the current speed of the robot (average of the 2 wheel
   * speeds).
   */
  double current_speed(void) const {
    auto tmp = reading();
    return (tmp.vel_left + tmp.vel_right) / 2.0;
  }

  void reset(void) {}

 private:
  /* clang-format off */
  TSensor* const m_sensor;
  /* clang-format on */
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using diff_drive_sensor = diff_drive_sensor_impl<argos::CCI_DifferentialSteeringSensor>;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
using diff_drive_sensor = diff_drive_sensor_impl<argos::CCI_PiPuckDifferentialDriveSensor>;
#else
class diff_drive_sensor{};
#endif /* COSM_HAL_TARGET */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_DIFF_DRIVE_SENSOR_HPP_ */
