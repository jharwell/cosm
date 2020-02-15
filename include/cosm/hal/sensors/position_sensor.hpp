/**
 * \file position_sensor.hpp
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

#ifndef INCLUDE_COSM_HAL_SENSORS_POSITION_SENSOR_HPP_
#define INCLUDE_COSM_HAL_SENSORS_POSITION_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include "rcppsw/math/vector2.hpp"
#include "cosm/cosm.hpp"

#include "cosm/hal/hal.hpp"

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#else
#error "Selected hardware has no position sensor!"
#endif /* HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_argos_position_sensor = std::is_same<TSensor,
                                           argos::CCI_PositioningSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class position_sensor_impl
 * \ingroup hal sensors
 *
 * \brief Position sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ARGoS footbot.
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                 HAL. If nullptr, then that effectively disables the sensor
 *                 at compile time, and SFINAE ensures no member functions can
 *                 be called.
 */
template <typename TSensor>
class position_sensor_impl {
 public:
  /**
   * \brief A position sensor reading, given as a 2D position and a triplet of
   * (X,Y,Z) angles.
   */
  struct sensor_reading {
    rmath::vector2d position{};
    rmath::radians x_ang{};
    rmath::radians y_ang{};
    rmath::radians z_ang{};
  };

  explicit position_sensor_impl(TSensor * const sensor) : m_sensor(sensor) {}

  /**
   * \brief Get the current position sensor readings for the footbot robot.
   *
   * \return A \ref sensor_reading.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_position_sensor<U>::value)>
  sensor_reading reading(void) const {
    auto tmp = m_sensor->GetReading();
    sensor_reading ret;
    argos::CRadians x, y, z;
    tmp.Orientation.ToEulerAngles(z, y, x);
    ret.x_ang = rmath::radians(x.GetValue());
    ret.y_ang = rmath::radians(y.GetValue());
    ret.z_ang = rmath::radians(z.GetValue());
    ret.position = rmath::vector2d(tmp.Position.GetX(), tmp.Position.GetY());
    return ret;
  }

 private:
  /* clang-format off */
  TSensor* const m_sensor;
  /* clang-format on */
};

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
using position_sensor = position_sensor_impl<argos::CCI_PositioningSensor>;
#endif /* HAL_TARGET */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_POSITION_SENSOR_HPP_ */
