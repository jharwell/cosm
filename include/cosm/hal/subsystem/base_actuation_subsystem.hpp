/**
 * \file base_actuation_subsystem.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
#include "cosm/hal/subsystem/actuator_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::subsystem {

/*******************************************************************************
 * Macros
 ******************************************************************************/
/**
 * \def COSM_HAL_ACTUATOR_ACCESSOR(type, name, ...)
 *
 * Shorthand for defining an accessor which returns a handle to an accessor in a
 * \ref chsubsystem::base_actuation_subsystem derived class.
 *
 * \param type The type of the accessor handle.
 *
 * \param name The name that the accessor function should have.
 *
 * \c const can be passed as an additional argument to make the accessor const.
 */
#define COSM_HAL_ACTUATOR_ACCESSOR(type, name, ...)           \
  COSM_HAL_SAA_ACCESSOR(actuator, type, name, __VA_ARGS__)

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_actuation_subsystem
 * \ingroup hal subsystem
 *
 * \brief Base actuation subsystem for all actuators used by all robot
 * controllers which actuate in 2D or 3D.
 */
template <typename ...TActuatorTypes>
class base_actuation_subsystem : private chsubsystem::base_subsystem {
 public:
  using variant_type = actuator_variant<TActuatorTypes...>;
  using actuator_map = actuator_variant_map<TActuatorTypes...>;

  /**
   * \brief Convenience function to create a actuator map create for the
   * specified actuator to make client code cleaner.
   */
  template <typename TActuator>
  static typename actuator_map::value_type map_entry_create(
      TActuator&& actuator) {
    return { typeid(TActuator), variant_type(std::move(actuator)) };
  }

  /**
   * \param actuators Map of handles to actuation devices, indexed by typeid.
   */
  explicit base_actuation_subsystem(actuator_map&& actuators)
      : m_actuators(std::move(actuators)) {}

  virtual ~base_actuation_subsystem(void) = default;

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
    return &std::get<T>(m_actuators.find(typeid(T))->second);
  }

  template <typename T>
  T* actuator(void) {
    return &std::get<T>(m_actuators.find(typeid(T))->second);
  }

 private:
  /* clang-format off */
  actuator_map m_actuators;
  /* clang-format off */
};

} /* namespace cosm::hal::subsystem */
