/**
 * @file light_sensor.hpp
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

#ifndef INCLUDE_COSM_HAL_SENSORS_LIGHT_SENSOR_HPP_
#define INCLUDE_COSM_HAL_SENSORS_LIGHT_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include "rcppsw/rcppsw.hpp"
#include "cosm/hal/hal.hpp"

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#else
#error "Selected hardware has no light sensor!"
#endif /* HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_argos_light_sensor = std::is_same<TSensor,
                                           argos::CCI_FootBotLightSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class light_sensor
 * @ingroup cosm hal sensors
 *
 * @brief Light sensor wrapper for the following supported robots:
 *
 * - ARGoS footbot. The simulated sensor is expensive to update each timestep,
 *   so it is disabled upon creation, so robots can selectively enable/disable
 *   it as needed for maximum speed.
 */
template <typename TSensor>
class _light_sensor {
 public:
  /**
   * @brief A light sensor reading (value, angle) pair.
   *
   * The first argument is the value of the sensor (distance to light), and the
   * second argument is the angle of the sensor on the robot in relation to the
   * positive x axis.
   */
  struct reading {
    double value;
    double angle;

    reading(double _value, double _angle) : value(_value), angle(_angle) {}
  };

  explicit _light_sensor(TSensor * const sensor) : m_sensor(sensor) {}

  /**
   * @brief Get the current light sensor readings for the footbot robot.
   *
   * @return A vector of \ref reading.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_light_sensor<U>::value)>
  std::vector<reading>  readings(void) const {
    std::vector<reading> ret;
    for (auto &r : m_sensor->GetReadings()) {
      ret.push_back({r.Value, r.Angle.GetValue()});
    } /* for(&r..) */

    return ret;
  }
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_light_sensor<U>::value)>
  void enable(void) const { m_sensor->Enable(); }
  void disable(void) const { m_sensor->Disable(); }

 private:
  TSensor* const m_sensor;
};

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
using light_sensor = _light_sensor<argos::CCI_FootBotLightSensor>;
#endif /* HAL_TARGET */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_LIGHT_SENSOR_HPP_ */
