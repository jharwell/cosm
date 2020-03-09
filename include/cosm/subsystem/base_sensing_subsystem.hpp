/**
 * \file base_sensing_subsystem.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SUBSYSTEM_BASE_SENSING_SUBSYSTEM_HPP_
#define INCLUDE_COSM_SUBSYSTEM_BASE_SENSING_SUBSYSTEM_HPP_

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
#include "cosm/hal/sensors/position_sensor.hpp"
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
 * \class base_sensing_subsystem
 * \ingroup subsystem
 *
 * \brief Base sensing subsystem for all sensors used by all robot
 * controllers. It uses a \ref hal::sensors::position_sensor for positioning
 * information, and manages any number of additional sensors:
 *
 * - \ref hal::sensors::proximity_sensor
 * - \ref hal::sensors::colored_blob_camera_sensor
 * - \ref hal::sensors::light_sensor
 * - \ref hal::sensors::ground_sensor
 * - \ref hal::sensors::battery_sensor
 * - \ref hal::sensors::diff_drive_sensor
 * - \ref hal::sensors::wifi_sensor
 */
class base_sensing_subsystem {
 public:
  using variant_type = boost::variant<hal::sensors::proximity_sensor,
                                      hal::sensors::wifi_sensor,
                                      hal::sensors::colored_blob_camera_sensor,
                                      hal::sensors::light_sensor,
                                      hal::sensors::ground_sensor,
                                      hal::sensors::battery_sensor,
                                      hal::sensors::diff_drive_sensor>;
  using sensor_map = std::map<std::type_index, variant_type>;


  /**
   * \param pos Position sensor.
   * \param sensors Map of handles to sensing devices, indexed by typeid.
   */
  base_sensing_subsystem(const hal::sensors::position_sensor& pos,
                         const sensor_map& sensors)
      : m_pos_sensor(pos), m_sensors(sensors) {}

  virtual ~base_sensing_subsystem(void) = default;

  /**
   * \brief Update the current time and position information for the robot.
   */
  virtual void update(const rtypes::timestep& t,
                      const rtypes::discretize_ratio& ratio) = 0;

  /**
   * \brief Convenience function to create a sensor map create for the
   * specified sensor to make client code cleaner.
   */
  template <typename TSensor>
  static sensor_map::value_type map_entry_create(const TSensor& sensor) {
    return {typeid(TSensor), variant_type(sensor)};
  }

  rtypes::timestep tick(void) const { return m_tick; }

  template <typename TSensor>
  bool replace(const TSensor& sensor) {
    if (m_sensors.end() != m_sensors.find(typeid(sensor))) {
      m_sensors.erase(m_sensors.find(typeid(sensor)));
      return m_sensors.insert(map_entry_create(sensor)).second;
    }
    return false;
  }

  template <typename T>
  const T* sensor(void) const {
    return &boost::get<T>(m_sensors.find(typeid(T))->second);
  }

  template <typename T>
  T* sensor(void) {
    return &boost::get<T>(m_sensors.find(typeid(T))->second);
  }

 protected:
  void tick(const rtypes::timestep& tick) { m_tick = tick; }
  const hal::sensors::position_sensor* pos_sensor(void) const {
    return &m_pos_sensor;
  }

 private:
  /* clang-format off */
  rtypes::timestep              m_tick{0};
  rmath::vector2u               m_dposition{};
  hal::sensors::position_sensor m_pos_sensor;
  sensor_map                    m_sensors;
  /* clang-format off */
};

NS_END(subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_BASE_SENSING_SUBSYSTEM _HPP_ */
