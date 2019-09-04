/**
 * @file actuation_subsystem2D.hpp
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

#ifndef INCLUDE_COSM_SUBSYSTEM_ACTUATION_SUBSYSTEM2D_HPP_
#define INCLUDE_COSM_SUBSYSTEM_ACTUATION_SUBSYSTEM2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>
#include <map>
#include <typeindex>

#include "cosm/hal/actuators/diff_drive_actuator.hpp"
#include "cosm/hal/actuators/led_actuator.hpp"
#include "cosm/hal/actuators/wifi_actuator.hpp"
#include "cosm/kin2D/diff_drive.hpp"
#include "cosm/kin2D/governed_diff_drive.hpp"
#include "cosm/steer2D/force_calculator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class actuation_subsystem2D
 * @ingroup cosm subsystem
 *
 * @brief The  actuation subsystem for all actuators used by robot
 * controllers that operate in 2D.
 *
 * Any controller with a sensing subsystem can choose any number of the
 * supported sensors to pass to the subsystem to manage:
 *
 * - \ref hal::led_actuator
 * - \ref hal::wifi_actuator
 *
 * In addition, the following "augmented" actuators can also be used, in order
 * to provide more functionality than the "raw" actuators can provide:
 *
 * - \ref kin2D::diff_drive
 * - \ref kin2D::governed_diff_drive
 */
class actuation_subsystem2D {
 public:
  using variant_type = boost::variant<hal::actuators::led_actuator,
                                      hal::actuators::wifi_actuator,
                                      kin2D::diff_drive,
                                      kin2D::governed_diff_drive>;

  using actuator_map = std::map<std::type_index, variant_type>;

  template <typename TActuator>
  static actuator_map::value_type map_entry_create(const TActuator& actuator) {
    return {typeid(TActuator), variant_type(actuator)};
  }

  /**
   * @param actuators Map of handles to actuator devices, indexed by typeid.
   */
  explicit actuation_subsystem2D(const actuator_map& actuators)
      : m_actuators(actuators) {}

  /**
   * @brief Reset all actuators, including stopping the robot.
   */
  void reset(void);

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
  /* clang-format on */
};

NS_END(subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_ACTUATION_SUBSYSTEM2D_HPP_ */
