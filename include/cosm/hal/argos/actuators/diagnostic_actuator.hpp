 /**
 * \file diagnostic_actuator.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>

#include "rcppsw/utils/color.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/rcppsw.hpp"

#include "cosm/hal/argos/actuators/led_actuator.hpp"
#include "cosm/hal/actuators/diagnostics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, argos, actuators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class diagnostic_actuator
 * \ingroup hal argos actuators
 *
 * \brief Diagnostic actuator via LEDs.
 *
 *  Supports the following robots:
 *
 * - ARGoS footbot
 * - ARGoS epuck
 * - ARGoS pipuck
 *
 * \tparam TActuator The underlying actuator handle type abstracted away by the
 *                   HAL.
 */
class diagnostic_actuator final : public rer::client<diagnostic_actuator>,
                                  public chargos::actuators::led_actuator {
 public:
  using chargos::actuators::led_actuator::set_color;
  using map_type = std::map<uint8_t, rutils::color>;

  explicit diagnostic_actuator(chargos::actuators::led_actuator::impl_type* leds,
                               const map_type& map)
      : ER_CLIENT_INIT("cosm.hal.argos.actuators.diagnostic"),
        chargos::actuators::led_actuator(leds),
        m_map(map) {}

  /* move only constructible/assignable for use with saa subsystem */
  const diagnostic_actuator& operator=(const diagnostic_actuator&) = delete;
  diagnostic_actuator(const diagnostic_actuator&) = delete;
  diagnostic_actuator& operator=(diagnostic_actuator&&) = default;
  diagnostic_actuator(diagnostic_actuator&&) = default;

  void emit(uint8_t type) {
    auto it = m_map.find(type);
    ER_ASSERT(it != m_map.end(),
              "Unknown diagnostic category %d",
              type);
    set_color(-1, it->second);
  }

  /* clang-format off */
  map_type m_map;
  /* clang-format on */
};

NS_END(actuators, argos, hal, cosm);
