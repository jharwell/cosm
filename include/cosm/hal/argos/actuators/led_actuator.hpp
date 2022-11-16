/**
 * \file led_actuator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/utils/color.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/argos/actuators/argos_actuator.hpp"

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
#include <argos3/plugins/robots/generic/control_interface/ci_directional_leds_actuator.h>
#endif /* COSM_HAL_TARGET */
#include <argos3/core/utility/datatypes/color.h>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_LEDsActuator;
class CCI_DirectionalLEDsActuator;
} /* namespace argos */

NS_START(cosm, hal, argos, actuators, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TActuator>
using is_generic_led_actuator = std::is_same<TActuator,
                                             ::argos::CCI_LEDsActuator>;

template<typename TActuator>
using is_generic_dirled_actuator = std::is_same<TActuator,
                                                ::argos::CCI_DirectionalLEDsActuator>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class led_actuator_impl
 * \ingroup hal argos actuators
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
 *                   HAL.
 */
template<typename TActuator>
class led_actuator_impl : public rer::client<led_actuator_impl<TActuator>>,
                          public chargos::actuators::argos_actuator<TActuator> {
 private:
  using chargos::actuators::argos_actuator<TActuator>::decoratee;

 public:
  using impl_type = TActuator;
  using chargos::actuators::argos_actuator<impl_type>::enable;
  using chargos::actuators::argos_actuator<impl_type>::disable;
  using chargos::actuators::argos_actuator<impl_type>::is_enabled;

  explicit led_actuator_impl(TActuator* const leds)
      : ER_CLIENT_INIT("cosm.hal.argos.actuators.led"),
        chargos::actuators::argos_actuator<TActuator>(leds) {}

  /* move only constructible/assignable for use with saa subsystem */
  const led_actuator_impl& operator=(const led_actuator_impl&) = delete;
  led_actuator_impl(const led_actuator_impl&) = delete;
  led_actuator_impl& operator=(led_actuator_impl&&) = default;
  led_actuator_impl(led_actuator_impl&&) = default;

  /**
   * \brief Reset the LED actuator (turn all LEDs off).
   */
  void reset(void) override {
    set_color(-1, rutils::color::kBLACK);
    argos_actuator<TActuator>::reset();
  }

  /**
   * \brief Set a single LED on the robot to a specific color (or set all LEDs
   * to a specific color).
   *
   * \param id Which LED to change color. This is application defined. However,
   * the reserved value of -1 should be interpreted to mean set the color of
   * \c ALL LEDs on the robot.
   *
   * \param color The color to change the LED to. This is application defined.
   *
   * \return \c TRUE if successful, and \c FALSE otherwise.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_generic_led_actuator<U>::value ||
                                  detail::is_generic_dirled_actuator<U>::value)>
  bool set_color(int id, const rutils::color& color) {
    ER_CHECK(nullptr != decoratee(),
             "%s called with NULL impl handle!",
             __FUNCTION__);
    ER_CHECK(is_enabled(),
             "%s called when disabled",
             __FUNCTION__);

    if (-1 == id) {
      decoratee()->SetAllColors(::argos::CColor(color.red(),
                                         color.green(),
                                         color.blue(),
                                         color.alpha()));
    } else {
      decoratee()->SetSingleColor(id, ::argos::CColor(color.red(),
                                               color.green(),
                                               color.blue(),
                                               color.alpha()));
    }
    return true;

 error:
    return false;
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
   *
   * \return \c TRUE if successful, and \c FALSE otherwise.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_generic_led_actuator<U>::value ||
                                  detail::is_generic_dirled_actuator<U>::value)>
  bool set_intensity(int id, uint8_t intensity) {
    ER_CHECK(nullptr != decoratee(),
             "%s called with NULL impl handle!",
             __FUNCTION__);
    ER_CHECK(is_enabled(),
             "%s called when disabled",
             __FUNCTION__);

    if (-1 == id) {
      decoratee()->SetAllIntensities(intensity);
    } else {
      decoratee()->SetSingleIntensity(id, intensity);
    }
    return true;

 error:
    return false;
  }
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using led_actuator = led_actuator_impl<::argos::CCI_LEDsActuator>;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
using led_actuator = led_actuator_impl<::argos::CCI_DirectionalLEDsActuator>;
#else
class led_actuator {};
#endif /* COSM_HAL_TARGET */

NS_END(actuators, argos, hal, cosm);
