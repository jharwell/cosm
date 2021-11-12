/**
 * \file proximity_sensor.hpp
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

#ifndef INCLUDE_COSM_HAL_ARGOS_SENSORS_PROXIMITY_SENSOR_HPP_
#define INCLUDE_COSM_HAL_ARGOS_SENSORS_PROXIMITY_SENSOR_HPP_

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
#include "cosm/hal/argos/sensors/config/proximity_sensor_config.hpp"
#include "cosm/hal/sensors/base_sensor.hpp"

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
using is_footbot_proximity_sensor = std::is_same<TSensor,
                                                       ::argos::CCI_FootBotProximitySensor>;
template<typename TSensor>
using is_epuck_proximity_sensor = std::is_same<TSensor,
                                                       ::argos::CCI_EPuckProximitySensor>;
template<typename TSensor>
using is_pipuck_proximity_sensor = std::is_same<TSensor,
                                                      std::false_type>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class proximity_sensor_impl
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
 *                  HAL. If nullptr, then that effectively disables the sensor
 *                  at compile time, and SFINAE ensures no member functions can
 *                  be called.
 */
template <typename TSensor>
class proximity_sensor_impl final : public rer::client<proximity_sensor_impl<TSensor>>,
                                    public chal::sensors::base_sensor<TSensor> {
 private:
  using chal::sensors::base_sensor<TSensor>::decoratee;

 public:
  using impl_type = TSensor;
  using chal::sensors::base_sensor<TSensor>::enable;
  using chal::sensors::base_sensor<TSensor>::disable;
  using chal::sensors::base_sensor<TSensor>::reset;

  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_footbot_proximity_sensor<U>::value ||
                               detail::is_epuck_proximity_sensor<U>::value)>
   proximity_sensor_impl(TSensor * const sensor,
                         const config::proximity_sensor_config* const config)
       : ER_CLIENT_INIT("cosm.hal.argos.sensors.proximity"),
         chal::sensors::base_sensor<TSensor>(sensor),
         mc_config(*config) {}

  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_pipuck_proximity_sensor<U>::value)>
  explicit proximity_sensor_impl(
      const config::proximity_sensor_config* const config)
       : ER_CLIENT_INIT("cosm.hal.sensors.proximity"),
         chal::sensors::base_sensor<TSensor>(nullptr),
         mc_config(*config) {}

  const proximity_sensor_impl& operator=(const proximity_sensor_impl&) = delete;
  proximity_sensor_impl(const proximity_sensor_impl&) = default;

  /**
   * \brief Return the average object reading within proximity for the
   * robot. Proximity is defined as:
   *
   * - Within the "go straight" angle range for the robot
   *
   * If there are not enough objects meeting this criteria that are close enough
   * such that the average distance to them is > than the provided delta,
   * nothing is returned.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_footbot_proximity_sensor<U>::value ||
                               detail::is_epuck_proximity_sensor<U>::value)>
  boost::optional<rmath::vector2d> avg_prox_obj(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);

    rmath::vector2d accum;

    size_t count = 0;

    /* constructed, not returned, so we need a copy to iterate over */
    auto readings = this->readings();
    for (auto& r : readings) {
      accum += r;
      ++count;
    }

    if (count == 0) {
      return boost::none;
    }

    accum /= count;

    if (mc_config.fov.contains(accum.angle()) &&
        accum.length() <= mc_config.delta) {
      return boost::none;
    } else {
      return boost::make_optional(accum);
    }
  }

  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_pipuck_proximity_sensor<U>::value)>
  boost::optional<rmath::vector2d> avg_prox_obj(void) const {
    return boost::optional<rmath::vector2d>();
  }

 private:
  /**
   * \brief Get the current proximity sensor readings for the footbot robot.
   *
   * \return A vector of (X,Y) pairs of sensor readings corresponding to
   * object distances.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_footbot_proximity_sensor<U>::value ||
                               detail::is_epuck_proximity_sensor<U>::value)>
  std::vector<rmath::vector2d> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);

    std::vector<rmath::vector2d> ret;
    for (auto &r : decoratee()->GetReadings()) {
      ret.emplace_back(r.Value, rmath::radians(r.Angle.GetValue()));
    } /* for(&r..) */

    return ret;
  }

  /* clang-format off */
  const config::proximity_sensor_config mc_config;
  /* clang-format on */
};

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT
using proximity_sensor = proximity_sensor_impl<::argos::CCI_FootBotProximitySensor>;
#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D
using proximity_sensor = proximity_sensor_impl<::argos::CCI_EPuckProximitySensor>;
#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK
using proximity_sensor = proximity_sensor_impl<std::false_type>;
#else
class proximity_sensor{};
#endif /* COSM_HAL_TARGET */

NS_END(sensors, argos, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ARGOS_SENSORS_PROXIMITY_SENSOR_HPP_ */
