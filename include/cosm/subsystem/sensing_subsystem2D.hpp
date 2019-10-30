/**
 * @file sensing_subsystem2D.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEM2D_HPP_
#define INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEM2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>
#include <map>
#include <typeindex>

#include "rcppsw/types/timestep.hpp"

#include "cosm/hal/sensors/battery_sensor.hpp"
#include "cosm/hal/sensors/colored_blob_camera_sensor.hpp"
#include "cosm/hal/sensors/diff_drive_sensor.hpp"
#include "cosm/hal/sensors/ground_sensor.hpp"
#include "cosm/hal/sensors/light_sensor.hpp"
#include "cosm/hal/sensors/proximity_sensor.hpp"
#include "cosm/hal/sensors/wifi_sensor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class sensing_subsystem2D
 * @ingroup cosm subsystem
 *
 * @brief The sensing subsystem for all sensors used by robotics controllers
 * that operate in 2D.
 *
 * Any controller with a sensing subsystem can choose any number of the
 * supported sensors to pass to the subsystem to manage:
 *
 * - \ref hal::sensors::proximity_sensor
 * - \ref hal::sensors::colored_blob_camera_sensor
 * - \ref hal::sensors::light_sensor
 * - \ref hal::sensors::ground_sensor
 * - \ref hal::sensors::battery_sensor
 * - \ref hal::sensors::diff_drive_sensor
 * - \ref hal::sensors::wifi_sensor
 */
class sensing_subsystem2D {
 public:
  using variant_type = boost::variant<hal::sensors::proximity_sensor,
                                      hal::sensors::wifi_sensor,
                                      hal::sensors::colored_blob_camera_sensor,
                                      hal::sensors::light_sensor,
                                      hal::sensors::ground_sensor,
                                      hal::sensors::battery_sensor,
                                      hal::sensors::diff_drive_sensor>;
  using sensor_map = std::map<std::type_index, variant_type>;

  template <typename TSensor>
  static sensor_map::value_type map_entry_create(const TSensor& sensor) {
    return {typeid(TSensor), variant_type(sensor)};
  }

  /**
   * @param sensors Map of handles to sensing devices, indexed by typeid.
   */
  explicit sensing_subsystem2D(sensor_map& sensors) : m_sensors(sensors) {}

  /**
   * @brief Get the robot's current location.
   *
   * @note This is set via an external process and that controller are *NOT*
   * capable of self-localizing. That's not the point of this project, and this
   * was much faster/easier.
   */
  const rmath::vector2d& position(void) const { return m_position; }
  const rmath::vector2u& discrete_position(void) const {
    return m_discrete_position;
  }

  void tick(const rtypes::timestep& t) { m_tick = t; }

  rtypes::timestep tick(void) const { return m_tick; }

  template <typename TSensor>
  bool replace(const TSensor& sensor) {
    if (m_sensors.end() != m_sensors.find(typeid(sensor))) {
      m_sensors.erase(m_sensors.find(typeid(sensor)));
      return m_sensors.insert(map_entry_create(sensor)).second;
    }
    return false;
  }

  /**
   * @brief Set the robot's current location.
   */
  void position(const rmath::vector2d& position) {
    m_prev_position = m_position;
    m_position = position;
  }

  void discrete_position(const rmath::vector2u& position) {
    m_discrete_position = position;
  }

  /**
   * @brief Get how far the robot has traveled in the last timestep, as well as
   * the direction/magnitude.
   */
  rmath::vector2d tick_travel(void) const {
    return m_position - m_prev_position;
  }

  /**
   * @brief Get the angle of the current robot's heading. A shortcut to help
   * reduce the ache in my typing fingers.
   *
   * @return The heading angle.
   */
  const rmath::radians& heading(void) const { return m_heading; }
  void heading(const rmath::radians& r) { m_heading = r; }

  template <typename T>
  const T* sensor(void) const {
    return &boost::get<T>(m_sensors.find(typeid(T))->second);
  }
  template <typename T>
  T* sensor(void) {
    return &boost::get<T>(m_sensors.find(typeid(T))->second);
  }

 private:
  /* clang-format off */
  rtypes::timestep m_tick{0};
  rmath::vector2d  m_position{};
  rmath::vector2d  m_prev_position{};
  rmath::radians   m_heading{};
  rmath::vector2u  m_discrete_position{};
  sensor_map       m_sensors;
  /* clang-format off */
};

NS_END(subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEM2D _HPP_ */
