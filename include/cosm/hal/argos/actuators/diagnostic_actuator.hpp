 /**
 * \file diagnostic_actuator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::hal::argos::actuators {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class diagnostic_actuator
 * \ingroup hal argos actuators
 *
 * \brief Diagnostic actuator via LEDs.
 *
 *  Can be used with any ARGoS robot supported by \ref
 *  chargos::actuators::led_acuator.
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

} /* namespace cosm::hal::argos::actuators */
