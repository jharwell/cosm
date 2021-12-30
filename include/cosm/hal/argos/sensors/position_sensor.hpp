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

#ifndef INCLUDE_COSM_HAL_ARGOS_SENSORS_POSITION_SENSOR_HPP_
#define INCLUDE_COSM_HAL_ARGOS_SENSORS_POSITION_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/cosm.hpp"
#include "cosm/hal/argos/sensors/argos_sensor.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/kin/pose.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, argos, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_position_sensor = std::is_same<TSensor,
                                           ::argos::CCI_PositioningSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class position_sensor_impl
 * \ingroup hal argos sensors
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
 *                 HAL. If nullptr, then that effectively disables the sensor.
 */
template <typename TSensor>
class position_sensor_impl final : public rer::client<position_sensor_impl<TSensor>>,
                                   public chargos::sensors::argos_sensor<TSensor> {
 private:
  using chargos::sensors::argos_sensor<TSensor>::decoratee;

 public:
  using impl_type = TSensor;
  using chargos::sensors::argos_sensor<impl_type>::enable;
  using chargos::sensors::argos_sensor<impl_type>::disable;
  using chargos::sensors::argos_sensor<impl_type>::reset;
  using chargos::sensors::argos_sensor<impl_type>::is_enabled;

  explicit position_sensor_impl(TSensor * const sensor)
      : ER_CLIENT_INIT("cosm.hal.argos.sensors.position"),
        chargos::sensors::argos_sensor<TSensor>(sensor) {}

  const position_sensor_impl& operator=(const position_sensor_impl&) = delete;
  position_sensor_impl(const position_sensor_impl&) = default;

  /**
   * \brief Get the current position sensor readings for the footbot robot.
   *
   * \return A \ref ckin::pose.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_position_sensor<U>::value)>
  ckin::pose reading(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    auto tmp = decoratee()->GetReading();
    ckin::pose ret;
    ::argos::CRadians x, y, z;
    tmp.Orientation.ToEulerAngles(z, y, x);
    ret.orientation = rmath::euler_angles(rmath::radians(x.GetValue()),
                                          rmath::radians(y.GetValue()),
                                          rmath::radians(z.GetValue()));
    ret.position = rmath::vector3d(tmp.Position.GetX(),
                                   tmp.Position.GetY(),
                                   tmp.Position.GetZ());
    return ret;
  }
};

using position_sensor = position_sensor_impl<::argos::CCI_PositioningSensor>;

NS_END(sensors, argos, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ARGOS_SENSORS_POSITION_SENSOR_HPP_ */
