/**
 * \file ir_sensor.hpp
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

#ifndef INCLUDE_COSM_HAL_ARGOS_SENSORS_IR_SENSOR_HPP_
#define INCLUDE_COSM_HAL_ARGOS_SENSORS_IR_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <limits>
#include <boost/optional.hpp>

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#endif /* COSM_HAL_TARGET */

#include "rcppsw/math/range.hpp"
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/argos/sensors/argos_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_FootBotProximitySensor;
class CCI_EPuckProximitySensor;
} /* namespace argos */

NS_START(cosm, hal, argos, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_footbot_ir_sensor = std::is_same<TSensor,
                                          ::argos::CCI_FootBotProximitySensor>;
template<typename TSensor>
using is_epuck_ir_sensor = std::is_same<TSensor,
                                        ::argos::CCI_EPuckProximitySensor>;
template<typename TSensor>
using is_pipuck_ir_sensor = std::is_same<TSensor, std::false_type>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ir_sensor_impl
 * \ingroup hal argos sensors
 *
 * \brief Proxmity sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ARGoS footbot
 * - ARGoS epuck
 * - ARGoS pipuck (stub only)
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor.
 */
template <typename TSensor>
class ir_sensor_impl : public rer::client<ir_sensor_impl<TSensor>>,
                       public chargos::sensors::argos_sensor<TSensor> {
 private:
  using chargos::sensors::argos_sensor<TSensor>::decoratee;

 public:
  using impl_type = TSensor;
  using chargos::sensors::argos_sensor<impl_type>::enable;
  using chargos::sensors::argos_sensor<impl_type>::disable;
  using chargos::sensors::argos_sensor<impl_type>::reset;
  using chargos::sensors::argos_sensor<impl_type>::is_enabled;

  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_footbot_ir_sensor<U>::value ||
                               detail::is_epuck_ir_sensor<U>::value)>
   ir_sensor_impl(TSensor * const sensor)
       : ER_CLIENT_INIT("cosm.hal.argos.sensors.ir"),
         chargos::sensors::argos_sensor<TSensor>(sensor) {}

  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_pipuck_ir_sensor<U>::value)>
   ir_sensor_impl(void)
       : ER_CLIENT_INIT("cosm.hal.sensors.ir"),
         chal::sensors::base_sensor<TSensor>(nullptr) {}

  const ir_sensor_impl& operator=(const ir_sensor_impl&) = delete;
  ir_sensor_impl(const ir_sensor_impl&) = default;

  /**
   * \brief Get the current ir sensor readings for the footbot robot.
   *
   * \return A vector of (X,Y) pairs of sensor readings corresponding to
   * object distances.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_footbot_ir_sensor<U>::value ||
                                  detail::is_epuck_ir_sensor<U>::value)>
  std::vector<rmath::vector2d> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);

    std::vector<rmath::vector2d> ret;
    for (auto &r : decoratee()->GetReadings()) {
      /*
       * Value of 0 means nothing in range, don't include it for further
       * processing--small optimization.
       */
      if (r.Value > 0) {
        ret.emplace_back(r.Value, rmath::radians(r.Angle.GetValue()));
      }
    } /* for(&r..) */

    return ret;
  }
};

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT
using ir_sensor = ir_sensor_impl<::argos::CCI_FootBotProximitySensor>;
#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D
using ir_sensor = ir_sensor_impl<::argos::CCI_EPuckProximitySensor>;
#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK
using ir_sensor = ir_sensor_impl<std::false_type>;
#else
class ir_sensor{};
#endif /* COSM_HAL_TARGET */

NS_END(sensors, argos, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ARGOS_SENSORS_IR_SENSOR_HPP_ */
