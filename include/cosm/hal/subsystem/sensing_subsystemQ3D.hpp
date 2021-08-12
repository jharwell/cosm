/**
 * \file sensing_subsystemQ3D.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_SUBSYSTEM_SENSING_SUBSYSTEMQ3D_HPP_
#define INCLUDE_COSM_HAL_SUBSYSTEM_SENSING_SUBSYSTEMQ3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <typeindex>
#include <unordered_map>
#include <boost/variant.hpp>

#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/hal/sensors/battery_sensor.hpp"
#include "cosm/hal/sensors/wifi_sensor.hpp"
#include "cosm/hal/sensors/diff_drive_sensor.hpp"
#include "cosm/hal/sensors/ground_sensor.hpp"
#include "cosm/hal/sensors/light_sensor.hpp"
#include "cosm/hal/sensors/position_sensor.hpp"
#include "cosm/hal/sensors/proximity_sensor.hpp"
#include "cosm/hal/sensors/colored_blob_camera_sensor.hpp"
#include "cosm/hal/subsystem/base_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, subsystem);

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define COSM_HAL_SENSOR_ACCESSOR(Typelist, type, name, ...)           \
  COSM_HAL_SAA_ACCESSOR(sensor, Typelist, type, name, __VA_ARGS__)

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT
#define COSM_HAL_ROBOT_SENSOR_TYPES             \
  hal::sensors::battery_sensor,                 \
    hal::sensors::wifi_sensor,                  \
    hal::sensors::light_sensor,                 \
    hal::sensors::ground_sensor,                \
    hal::sensors::diff_drive_sensor,            \
    hal::sensors::proximity_sensor,             \
    hal::sensors::colored_blob_camera_sensor,   \
    hal::sensors::position_sensor

#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D
#define COSM_HAL_ROBOT_SENSOR_TYPES             \
  hal::sensors::light_sensor,                   \
    hal::sensors::ground_sensor,                \
    hal::sensors::diff_drive_sensor,            \
    hal::sensors::proximity_sensor,             \
    hal::sensors::colored_blob_camera_sensor,   \
    hal::sensors::position_sensor
#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK
#define COSM_HAL_ROBOT_SENSOR_TYPES             \
    hal::sensors::ground_sensor,                \
      hal::sensors::proximity_sensor,           \
    hal::sensors::diff_drive_sensor,            \
    hal::sensors::colored_blob_camera_sensor,   \
    hal::sensors::position_sensor
#endif

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sensing_subsystemQ3D
 * \ingroup hal subsystem
 *
 * \brief Base sensing subsystem for all sensors used by all robot controllers
 * which actuate in 2D, but can sense in 3D (quasi-3D). It uses a \ref
 * hal::sensors::position_sensor for positioning information, and manages any
 * number of additional sensors, depending on which robot the HAL is built for:
 *
 * - \ref hal::sensors::proximity_sensor
 * - \ref hal::sensors::colored_blob_camera_sensor
 * - \ref hal::sensors::light_sensor
 * - \ref hal::sensors::position_sensor
 * - \ref hal::sensors::ground_sensor
 * - \ref hal::sensors::battery_sensor
 * - \ref hal::sensors::diff_drive_sensor
 * - \ref hal::sensors::wifi_sensor
 */
class sensing_subsystemQ3D : private chsubsystem::base_subsystem {
 public:
  using variant_type = boost::variant<COSM_HAL_ROBOT_SENSOR_TYPES>;
  using sensor_map = std::unordered_map<std::type_index, variant_type>;

  /**
   * \brief Convenience function to create a sensor map create for the
   * specified sensor to make client code cleaner.
   */
  template <typename TSensor>
  static sensor_map::value_type map_entry_create(const TSensor& sensor) {
    return { typeid(TSensor), variant_type(sensor) };
  }

  /**
   * \param sensors Map of handles to sensing devices, indexed by typeid.
   */
  explicit sensing_subsystemQ3D(const sensor_map& sensors)
      : m_sensors(sensors) {}

  virtual ~sensing_subsystemQ3D(void) = default;

  /**
   * \brief Reset all sensors.
   */
  void reset(void) { base_subsystem::reset(m_sensors); }

  /**
   * \brief Disable all sensors.
   */
  void disable(void) { base_subsystem::disable(m_sensors); }

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

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::proximity_sensor,
                           proximity);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::proximity_sensor,
                           proximity,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::colored_blob_camera_sensor,
                           blobs);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::colored_blob_camera_sensor,
                           blobs,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::light_sensor,
                           light);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::light_sensor,
                           light,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::ground_sensor,
                           ground);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::ground_sensor,
                           ground,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::battery_sensor,
                           battery);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::battery_sensor,
                           battery,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::diff_drive_sensor,
                           diff_drive);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::diff_drive_sensor,
                           diff_drive,
                           const);

  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::position_sensor,
                           position);
  COSM_HAL_SENSOR_ACCESSOR(robot_sensor_types,
                           hal::sensors::position_sensor,
                           position,
                           const);

 private:
  /* clang-format off */
  sensor_map m_sensors;
  /* clang-format off */
};

NS_END(subsystem, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SUBSYSTEM_SENSING_SUBSYSTEMQ3D _HPP_ */
