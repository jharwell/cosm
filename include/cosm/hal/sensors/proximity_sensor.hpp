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

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#else
#error "Selected hardware has no proximity sensor!"
#endif /* HAL_TARGET */

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
 * \class proximity_sensor_impl
 * \ingroup hal sensors
 *
 * \brief Proxmity sensor wrapper.
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
class proximity_sensor_impl {
 public:
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_proximity_sensor<U>::value)>
   proximity_sensor_impl(TSensor * const sensor,
                     const config::proximity_sensor_config* const config)
      : mc_config(*config),
        m_sensor(sensor) {}

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

  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_proximity_sensor<U>::value)>
  void enable(void) const { m_sensor->Enable(); }

  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_proximity_sensor<U>::value)>
  void disable(void) const { m_sensor->Disable(); }

 private:
  /**
   * \brief Get the current proximity sensor readings for the footbot robot.
   *
   * \return A vector of (X,Y) pairs of sensor readings corresponding to
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

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
using proximity_sensor = proximity_sensor_impl<argos::CCI_FootBotProximitySensor>;
#endif /* HAL_TARGET */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_PROXIMITY_SENSOR_HPP_ */
