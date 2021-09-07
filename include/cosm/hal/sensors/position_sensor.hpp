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

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/cosm.hpp"
#include "cosm/hal/hal.hpp"

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

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
class position_sensor_impl final : public rer::client<position_sensor_impl<TSensor>>,
                                   private rpdecorator::decorator<TSensor*> {
 private:
  using rpdecorator::decorator<TSensor*>::decoratee;

 public:
  using impl_type = TSensor;

  /**
   * \brief A position sensor reading, given as a 2D position and a triplet of
   * (X,Y,Z) angles.
   */
  struct sensor_reading {
    rmath::vector3d position{};
    rmath::radians x_ang{};
    rmath::radians y_ang{};
    rmath::radians z_ang{};
  };

  explicit position_sensor_impl(TSensor * const sensor)
      : ER_CLIENT_INIT("cosm.hal.sensors.position"),
        rpdecorator::decorator<TSensor*>(sensor) {}

  const position_sensor_impl& operator=(const position_sensor_impl&) = delete;
  position_sensor_impl(const position_sensor_impl&) = default;

  /**
   * \brief Get the current position sensor readings for the footbot robot.
   *
   * \return A \ref sensor_reading.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_position_sensor<U>::value)>
  sensor_reading reading(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);

    auto tmp = decoratee()->GetReading();
    sensor_reading ret;
    argos::CRadians x, y, z;
    tmp.Orientation.ToEulerAngles(z, y, x);
    ret.x_ang = rmath::radians(x.GetValue());
    ret.y_ang = rmath::radians(y.GetValue());
    ret.z_ang = rmath::radians(z.GetValue());
    ret.position = rmath::vector3d(tmp.Position.GetX(),
                                   tmp.Position.GetY(),
                                   tmp.Position.GetZ());
    return ret;
  }

  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_position_sensor<U>::value)>
  void reset(void) { decoratee()->Reset(); }

template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_position_sensor<U>::value)>
  void disable(void) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    decoratee()->Disable();
  }


};

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using position_sensor = position_sensor_impl<argos::CCI_PositioningSensor>;
#else
class position_sensor{};
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_POSITION_SENSOR_HPP_ */
