/**
 * \file wifi_sensor.hpp
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

#ifndef INCLUDE_COSM_HAL_ARGOS_SENSORS_WIFI_SENSOR_HPP_
#define INCLUDE_COSM_HAL_ARGOS_SENSORS_WIFI_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/wifi_packet.hpp"
#include "cosm/hal/sensors/base_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_RangeAndBearingSensor;
} /* namespace argos */

NS_START(cosm, hal, argos, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_sensor = std::is_same<TSensor, ::argos::CCI_RangeAndBearingSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class wifi_sensor_impl
 * \ingroup hal argos sensors
 *
 * \brief Wireless communication sensor with range and bearing information for
 * the following supported robots:
 *
 * - ARGoS footbot
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor
 *                  at compile time, and SFINAE ensures no member functions can
 *                  be called.
 */
template <typename TSensor>
class wifi_sensor_impl final : public rer::client<wifi_sensor_impl<TSensor>>,
                               public chal::sensors::base_sensor<TSensor> {
 private:
  using chal::sensors::base_sensor<TSensor>::decoratee;

 public:
  using impl_type = TSensor;
  using chal::sensors::base_sensor<TSensor>::enable;
  using chal::sensors::base_sensor<TSensor>::disable;
  using chal::sensors::base_sensor<TSensor>::reset;

  explicit wifi_sensor_impl(TSensor * const sensor)
      : ER_CLIENT_INIT("cosm.hal.argos.sensors.wifi"),
        chal::sensors::base_sensor<TSensor>(sensor) {}

  const wifi_sensor_impl& operator=(const wifi_sensor_impl&) = delete;
  wifi_sensor_impl(const wifi_sensor_impl&) = default;

  /**
   * \brief Get the current rab wifi sensor readings for the footbot robot.
   *
   * \return A vector of \ref wifi_packet.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_sensor<U>::value)>
  std::vector<wifi_packet> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);

    std::vector<wifi_packet> ret;
    for (auto &r : decoratee()->GetReadings()) {
      wifi_packet d;
      for (size_t i = 0; i < r.Data.Size(); ++i) {
        d.data.push_back(r.Data[i]);
      } /* for(i..) */
      ret.push_back(d);
    } /* for(&r..) */
    return ret;
  }
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
using wifi_sensor = wifi_sensor_impl<::argos::CCI_RangeAndBearingSensor>;
#else
struct wifi_sensor{};
#endif /* COSM_HAL_TARGET */

NS_END(sensors, argos, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ARGOS_SENSORS_WIFI_SENSOR_HPP_ */
