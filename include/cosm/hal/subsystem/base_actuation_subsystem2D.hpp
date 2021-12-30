/**
 * \file base_actuation_subsystemQ3D.hpp
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

#ifndef INCLUDE_COSM_HAL_SUBSYSTEM_BASE_ACTUATION_SUBSYSTEM2D_HPP_
#define INCLUDE_COSM_HAL_SUBSYSTEM_BASE_ACTUATION_SUBSYSTEM2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <typeindex>
#include <unordered_map>
#include <boost/variant.hpp>

#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/hal/subsystem/base_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, subsystem);

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define COSM_HAL_ACTUATOR_ACCESSOR(Typelist, type, name, ...)           \
  COSM_HAL_SAA_ACCESSOR(actuator, Typelist, type, name, __VA_ARGS__)


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_actuation_subsystem2D
 * \ingroup hal subsystem
 *
 * \brief Base actuation subsystem for all actuators used by all robot
 * controllers which actuate in 2D.
 */
template <typename ...TActuatorTypes>
class base_actuation_subsystem2D : private chsubsystem::base_subsystem {
 public:
  using variant_type = boost::variant<TActuatorTypes...>;
  using actuator_map = std::unordered_map<std::type_index, variant_type>;

  /**
   * \brief Convenience function to create a actuator map create for the
   * specified actuator to make client code cleaner.
   */
  template <typename TActuator>
  static typename actuator_map::value_type map_entry_create(
      const TActuator& actuator) {
    return { typeid(TActuator), variant_type(actuator) };
  }

  /**
   * \param actuators Map of handles to actuation devices, indexed by typeid.
   */
  explicit base_actuation_subsystem2D(const actuator_map& actuators)
      : m_actuators(actuators) {}

  virtual ~base_actuation_subsystem2D(void) = default;

  /**
   * \brief Reset all actuators.
   */
  void reset(void) { base_subsystem::reset(m_actuators); }

  /**
   * \brief Disable all actuators.
   */
  void disable(void) { base_subsystem::disable(m_actuators); }

  template <typename TActuator>
  bool replace(const TActuator& actuator) {
    if (m_actuators.end() != m_actuators.find(typeid(actuator))) {
      m_actuators.erase(m_actuators.find(typeid(actuator)));
      return m_actuators.insert(map_entry_create(actuator)).second;
    }
    return false;
  }

  template <typename T>
  const T* actuator(void) const {
    return &boost::get<T>(m_actuators.find(typeid(T))->second);
  }

  template <typename T>
  T* actuator(void) {
    return &boost::get<T>(m_actuators.find(typeid(T))->second);
  }

 private:
  /* clang-format off */
  actuator_map m_actuators;
  /* clang-format off */
};

NS_END(subsystem, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SUBSYSTEM_BASE_ACTUATION_SUBSYSTEM2D _HPP_ */
