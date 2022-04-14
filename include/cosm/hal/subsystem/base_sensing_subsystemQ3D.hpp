/**
 * \file base_sensing_subsystemQ3D.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <typeindex>
#include <unordered_map>
#include <variant>
#include <utility>

#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/hal/subsystem/base_subsystem.hpp"
#include "cosm/hal/subsystem/sensor_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, subsystem);

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define COSM_HAL_SENSOR_ACCESSOR(Typelist, type, name, ...)           \
  COSM_HAL_SAA_ACCESSOR(sensor, Typelist, type, name, __VA_ARGS__)


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_sensing_subsystemQ3D
 * \ingroup hal subsystem
 *
 * \brief Base sensing subsystem for all sensors used by all robot controllers
 * which actuate in 2D, but can sense in 3D (quasi-3D).
 */
template <typename ...TSensorTypes>
class base_sensing_subsystemQ3D : private chsubsystem::base_subsystem {
 public:
  using sensor_map = sensor_variant_map<TSensorTypes...>;
  using variant_type = sensor_variant<TSensorTypes...>;

  /**
   * \brief Convenience function to create a sensor map create for the
   * specified sensor to make client code cleaner.
   */
  template <typename TSensor>
  static typename sensor_map::value_type map_entry_create(
      TSensor&& sensor) {
    return {typeid(TSensor), variant_type(std::move(sensor))};
  }

  /**
   * \param sensors Map of handles to sensing devices, indexed by typeid.
   */
  explicit base_sensing_subsystemQ3D(sensor_map&& sensors)
      : m_sensors(std::move(sensors)) {}

  virtual ~base_sensing_subsystemQ3D(void) = default;

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
    return &std::get<T>(m_sensors.find(typeid(T))->second);
  }

  template <typename T>
  T* sensor(void) {
    return &std::get<T>(m_sensors.find(typeid(T))->second);
  }

 private:
  /* clang-format off */
  sensor_map m_sensors;
  /* clang-format off */
};

NS_END(subsystem, hal, cosm);
