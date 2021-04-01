/**
 * \file wifi_actuator.hpp
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

#ifndef INCLUDE_COSM_HAL_ACTUATORS_WIFI_ACTUATOR_HPP_
#define INCLUDE_COSM_HAL_ACTUATORS_WIFI_ACTUATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/utils/color.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/hal/wifi_packet.hpp"

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#else
#error "Selected component has no RAB actuator!"
#endif /* COSM_HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, actuators, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename Actuator>
using is_argos_rab_actuator = std::is_same<Actuator,
                                           argos::CCI_RangeAndBearingActuator>;
NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class wifi_actuator_impl
 * \ingroup hal actuators
 *
 * \brief WIFI actuator wrapper.
 *
 * Supports the following robots:
 *
 * - ARGoS footbot. These robots will use wifi to broadcast data every timestep
 *   to all robots in range until told to do otherwise.
 *
 * - ARGoS epuck.  These robots will use wifi to broadcast data every timestep
 *   to all robots in range until told to do otherwise.
 *
 * \tparam TActuator The underlying actuator handle type abstracted away by the
 *                   HAL. If nullptr, then that effectively disables the
 *                   actuator at compile time, and SFINAE ensures no member
 *                   functions can be called.
 */
template<typename TActuator>
class wifi_actuator_impl {
 public:
  using impl_type = TActuator;

  explicit wifi_actuator_impl(TActuator* const wifi) : m_wifi(wifi) {}

  /**
   * \brief Start broadcasting the specified data to all footbots within range.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_rab_actuator<U>::value)>
  void broadcast_start(const struct wifi_packet& packet) {
    for (size_t i = 0; i < packet.data.size(); ++i) {
      m_wifi->SetData(i, packet.data[i]);
    } /* for(i..) */
  }

  /**
   * \brief Stop broadcasting the previously specified data to all footbots
   * within range.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_rab_actuator<U>::value)>
  void broadcast_stop(void) {
    m_wifi->ClearData();
  }

  /**
   * \brief Reset the wifi device.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_rab_actuator<U>::value)>
  void reset(void) {
    if (nullptr != m_wifi) {
      m_wifi->ClearData();
    }
  }

 private:
  /* clang-format off */
  TActuator* const m_wifi;
  /* clang-format on */
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using wifi_actuator = wifi_actuator_impl<argos::CCI_RangeAndBearingActuator>;
#endif /* COSM_HAL_TARGET */

NS_END(actuators, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ACTUATORS_WIFI_ACTUATOR_HPP_ */
