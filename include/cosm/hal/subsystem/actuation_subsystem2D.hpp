/**
 * \file actuation_subsystem2D.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_SUBSYSTEM_ACTUATION_SUBSYSTEM2D_HPP_
#define INCLUDE_COSM_HAL_SUBSYSTEM_ACTUATION_SUBSYSTEM2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <typeindex>
#include <boost/variant.hpp>

#include "rcppsw/mpl/typelist.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/hal/subsystem/base_subsystem.hpp"

#include "cosm/kin2D/diff_drive.hpp"
#include "cosm/kin2D/governed_diff_drive.hpp"
#include "cosm/hal/actuators/led_actuator.hpp"
#include "cosm/hal/actuators/wifi_actuator.hpp"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define COSM_HAL_ACTUATOR_ACCESSOR(Typelist, type, name, ...)   \
  COSM_HAL_SAA_ACCESSOR(actuator, Typelist, type, name, __VA_ARGS__)

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT
#define COSM_HAL_ROBOT_ACTUATOR_TYPES           \
  chargos::actuators::led_actuator,          \
    chargos::actuators::wifi_actuator,       \
    kin2D::diff_drive,                          \
    kin2D::governed_diff_drive,                 \
    chal::actuators::diff_drive_actuator

#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D
#define COSM_HAL_ROBOT_ACTUATOR_TYPES              \
  chal::actuators::led_actuator,                    \
    kin2D::diff_drive,                             \
    kin2D::governed_diff_drive,                    \
    chal::actuators::diff_drive_actuator

#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK
#define COSM_HAL_ROBOT_ACTUATOR_TYPES           \
  chal::actuators::led_actuator,                 \
    kin2D::diff_drive,                          \
    kin2D::governed_diff_drive,                 \
    chal::actuators::diff_drive_actuator
#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_TURTLEBOT3
#define COSM_HAL_ROBOT_ACTUATOR_TYPES           \
    kin2D::diff_drive,                          \
    kin2D::governed_diff_drive
#endif

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class actuation_subsystem2D
 * \ingroup hal subsystem
 *
 * \brief The actuation subsystem for any robot which operates in 2D.
 */
class actuation_subsystem2D : private chsubsystem::base_subsystem {
 public:
  using variant_type = boost::variant<COSM_HAL_ROBOT_ACTUATOR_TYPES>;
  using actuator_map = std::map<std::type_index, variant_type>;

  template <typename TActuator>
  static actuator_map::value_type map_entry_create(const TActuator& actuator) {
    return { typeid(TActuator), variant_type(actuator) };
  }

  /**
   * \param actuators Map of handles to actuator devices, indexed by typeid.
   */
  explicit actuation_subsystem2D(const actuator_map& actuators)
      : m_actuators(actuators) {}

  /**
   * \brief Reset all actuators.
   */
  void reset(void) { base_subsystem::reset(m_actuators); }

  /**
   * \brief Disable all actuators.
   */
  void disable(void) { base_subsystem::disable(m_actuators); }

  template <typename T>
  const T* actuator(void) const {
    return &boost::get<T>(m_actuators.find(typeid(T))->second);
  }
  template <typename T>
  T* actuator(void) {
    return &boost::get<T>(m_actuators.find(typeid(T))->second);
  }

  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             kin2D::governed_diff_drive,
                             governed_diff_drive);
  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             kin2D::governed_diff_drive,
                             governed_diff_drive,
                             const);

  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chal::actuators::led_actuator,
                             leds);
  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chal::actuators::led_actuator,
                             leds,
                             const);

  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chal::actuators::wifi_actuator,
                             rab);
  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chal::actuators::wifi_actuator,
                             rab,
                             const);

 private:
  /* clang-format off */
  actuator_map m_actuators;
  /* clang-format on */
};

NS_END(subsystem, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SUBSYSTEM_ACTUATION_SUBSYSTEM2D_HPP_ */
