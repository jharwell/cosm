/**
 * @file wifi_sensor.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_SENSORS_WIFI_SENSOR_HPP_
#define INCLUDE_COSM_HAL_SENSORS_WIFI_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include "cosm/hal/wifi_packet.hpp"
#include "rcppsw/math/radians.hpp"

#if HAL_CONFIG == HAL_CONFIG_ARGOS_FOOTBOT
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#else
#error "Selected hardware has no RAB wireless communication sensor!"
#endif /* HAL_CONFIG */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_argos_sensor = std::is_same<TSensor,
                                         argos::CCI_RangeAndBearingSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class wifi_sensor
 * @ingroup cosm hal sensors
 *
 * @brief Wireless communication sensor with range and bearing information for
 * the following supported robots:
 *
 * - ARGoS footbot
 */
template <typename TSensor>
class _wifi_sensor  {
 public:
  explicit _wifi_sensor(TSensor * const sensor) : m_sensor(sensor) {}

  /**
   * @brief Get the current rab wifi sensor readings for the footbot robot.
   *
   * @return A vector of \ref wifi_packet.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_sensor<U>::value)>
  std::vector<wifi_packet> readings(void) const {
    std::vector<wifi_packet> ret;
    for (auto &r : m_sensor->GetReadings()) {
      wifi_packet d;
      for (size_t i = 0; i < r.Data.Size(); ++i) {
        d.data.push_back(r.Data[i]);
      } /* for(i..) */
      ret.push_back(d);
    } /* for(&r..) */
    return ret;
  }

 private:
  TSensor* m_sensor;
};

#if HAL_CONFIG == HAL_CONFIG_ARGOS_FOOTBOT
using wifi_sensor = _wifi_sensor<argos::CCI_RangeAndBearingSensor>;
#endif /* HAL_CONFIG */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_WIFI_SENSOR_HPP_ */
