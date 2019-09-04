/**
 * @file proximity_sensor.hpp
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

#ifndef INCLUDE_COSM_HAL_SENSORS_PROXIMITY_SENSOR_HPP_
#define INCLUDE_COSM_HAL_SENSORS_PROXIMITY_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <limits>
#include <boost/optional.hpp>

#include "rcppsw/rcppsw.hpp"
#include "cosm/hal/hal.hpp"
#include "rcppsw/math/range.hpp"
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector2.hpp"
#include "cosm/hal/sensors/config/proximity_sensor_config.hpp"

#if HAL_CONFIG == HAL_CONFIG_ARGOS_FOOTBOT
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#else
#error "Selected hardware has no proximity sensor!"
#endif /* HAL_CONFIG */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_argos_proximity_sensor = std::is_same<TSensor,
                                               argos::CCI_FootBotProximitySensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class proximity_sensor
 * @ingroup cosm hal sensors
 *
 * @brief Proxmity sensor wrapper for the following supported robots:
 *
 * - ARGoS footbot
 */
template <typename TSensor>
class _proximity_sensor {
 public:
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_proximity_sensor<U>::value)>
   _proximity_sensor(TSensor * const sensor,
                     const config::proximity_sensor_config* const config)
      : mc_config(*config),
        m_sensor(sensor) {}

  const _proximity_sensor& operator=(const _proximity_sensor&) = delete;
  _proximity_sensor(const _proximity_sensor&) = default;

  /**
   * @brief Return the average object reading within proximity for the
   * robot. Proximity is defined as:
   *
   * - Within the "go straight" angle range for the robot
   *
   * If there are not enough objects meeting this criteria that are close enough
   * such that the average distance to them is > than the provided delta,
   * nothing is returned
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_proximity_sensor<U>::value)>
  boost::optional<rmath::vector2d> avg_prox_obj(void) const {
    rmath::vector2d accum;
    for (auto& r : readings()) {
      accum += r;
    }
    if (mc_config.fov.contains(accum.angle()) &&
        accum.length() <= mc_config.delta) {
      return boost::optional<rmath::vector2d>();
    } else {
      return boost::make_optional(accum);
    }
  }

 private:
  /**
   * @brief Get the current proximity sensor readings for the footbot robot.
   *
   * @return A vector of (X,Y) pairs of sensor readings corresponding to
   * object distances.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_proximity_sensor<U>::value)>
  std::vector<rmath::vector2d> readings(void) const {
    std::vector<rmath::vector2d> ret;
    for (auto &r : m_sensor->GetReadings()) {
      ret.emplace_back(r.Value, rmath::radians(r.Angle.GetValue()));
    } /* for(&r..) */

    return ret;
  }

  /* clang-format off */
  const config::proximity_sensor_config mc_config;
  TSensor* const                        m_sensor;
  /* clang-format on */
};

#if HAL_CONFIG == HAL_CONFIG_ARGOS_FOOTBOT
using proximity_sensor = _proximity_sensor<argos::CCI_FootBotProximitySensor>;
#endif /* HAL_CONFIG */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_PROXIMITY_SENSOR_HPP_ */
