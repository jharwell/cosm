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
#include "rcppsw/utils/color.hpp"
#include "rcppsw/er/client.hpp"

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

  explicit diagnostic_actuator(chargos::actuators::led_actuator::impl_type* leds)
      : ER_CLIENT_INIT("cosm.hal.argos.actuators.diagnostic"),
        chargos::actuators::led_actuator(leds) {}

  /* move only constructible/assignable for use with saa subsystem */
  const diagnostic_actuator& operator=(const diagnostic_actuator&) = delete;
  diagnostic_actuator(const diagnostic_actuator&) = delete;
  diagnostic_actuator& operator=(diagnostic_actuator&&) = default;
  diagnostic_actuator(diagnostic_actuator&&) = default;

  void emit(const chactuators::diagnostics& type) {
    switch (type) {
      case chactuators::diagnostics::ekEXPLORE:
        set_color(-1, rutils::color::kMAGENTA);
        break;
      case chactuators::diagnostics::ekSUCCESS:
        set_color(-1, rutils::color::kGREEN);
        break;
      case chactuators::diagnostics::ekTAXIS:
        set_color(-1, rutils::color::kYELLOW);
        break;
      case chactuators::diagnostics::ekLEAVING_NEST:
        set_color(-1, rutils::color::kGRAY50);
        break;
      case chactuators::diagnostics::ekWAIT_FOR_SIGNAL:
        set_color(-1, rutils::color::kWHITE);
        break;
      case chactuators::diagnostics::ekVECTOR_TO_GOAL:
        set_color(-1, rutils::color::kBLUE);
        break;
      case chactuators::diagnostics::ekEXP_INTERFERENCE:
        set_color(-1, rutils::color::kRED);
        break;
      default:
        ER_FATAL_SENTINEL("Unknown diagnostic category %d",
                          rcppsw::as_underlying(type));
    } /* switch() */
  }
};

NS_END(actuators, argos, hal, cosm);
