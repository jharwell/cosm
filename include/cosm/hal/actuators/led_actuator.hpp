/**
 * \file led_actuator.hpp
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

#ifndef INCLUDE_COSM_HAL_ACTUATORS_LED_ACTUATOR_HPP_
#define INCLUDE_COSM_HAL_ACTUATORS_LED_ACTUATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/utils/color.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/cosm.hpp"
#include "rcsw/common/fpc.h"

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
#include <argos3/plugins/robots/pi-puck/control_interface/ci_directional_leds_actuator.h>
#endif /* COSM_HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_LEDsActuator;
class CCI_DirectionalLEDsActuator;
} /* namespace argos */

NS_START(cosm, hal, actuators, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TActuator>
using is_argos_generic_led_actuator = std::is_same<TActuator,
                                                   argos::CCI_LEDsActuator>;

template<typename TActuator>
using is_argos_generic_dirled_actuator = std::is_same<TActuator,
                                                      argos::CCI_DirectionalLEDsActuator>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class led_actuator_impl
 * \ingroup hal
 *
 * \brief LED actuator wrapper.
 *
 *  Supports the following robots:
 *
 * - ARGoS footbot
 * - ARGoS epuck
 * - ARGoS pipuck
 *
 * \tparam TActuator The underlying actuator handle type abstracted away by the
 *                   HAL. If nullptr, then that effectively disables the
 *                   actuator at compile time, and SFINAE ensures no member
 *                   functions can be called.
 */
template<typename TActuator>
class led_actuator_impl {
 public:
  using impl_type = TActuator;

  explicit led_actuator_impl(TActuator* const leds) : m_leds(leds) {}

  /**
   * \brief Reset the LED actuator (turn all LEDs off).
   */
  void reset(void) { set_color(-1, rutils::color::kBLACK); }

  /**
   * \brief Set a single LED on the robot to a specific color (or set all LEDs
   * to a specific color).
   *
   * \param id Which LED to change color. This is application defined. However,
   * the reserved value of -1 should be interpreted to mean set the color of
   * \c ALL LEDs on the robot.
   *
   * \param color The color to change the LED to. This is application defined.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_generic_led_actuator<U>::value ||
                                  detail::is_argos_generic_dirled_actuator<U>::value)>
  void set_color(int id, const rutils::color& color) {
    RCPPSW_FPC_RET_V(nullptr != m_leds);

    if (-1 == id) {
      m_leds->SetAllColors(argos::CColor(color.red(),
                                         color.green(),
                                         color.blue(),
                                         color.alpha()));
    } else {
      m_leds->SetSingleColor(id, argos::CColor(color.red(),
                                               color.green(),
                                               color.blue(),
                                               color.alpha()));
    }
  }

  /**
   * \brief Set intensity for a single LED on the robot (or set intensity of all
   * LEDs).
   *
   * \param id Which LED to set intensity for. This is application
   * defined. However, the reserved value of -1 should be interpreted to mean
   * set the intensity of \c ALL LEDs on the robot.
   *
   * \param intensity In the range [0,255]. Application defined meaning.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_generic_led_actuator<U>::value ||
                                  detail::is_argos_generic_dirled_actuator<U>::value)>
  void set_intensity(int id, uint8_t intensity) {
    RCPPSW_FPC_RET_V(nullptr != m_leds);

    if (-1 == id) {
      m_leds->SetAllIntensities(intensity);
    } else {
      m_leds->SetSingleIntensity(id, intensity);
    }
  }

 private:
  /* clang-format off */
  TActuator* m_leds;
  /* clang-format on */
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using led_actuator = led_actuator_impl<argos::CCI_LEDsActuator>;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
using led_actuator = led_actuator_impl<argos::CCI_DirectionalLEDsActuator>;
#else
class led_actuator {};
#endif /* COSM_HAL_TARGET */

NS_END(actuators, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ACTUATORSLED_ACTUATOR_IMPL_HPP_ */
