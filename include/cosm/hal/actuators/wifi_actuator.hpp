/**
 * @file wifi_actuator.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#if HAL_CONFIG == HAL_CONFIG_ARGOS_FOOTBOT
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#else
#error "Selected component has no RAB actuator!"
#endif /* HAL_CONFIG */

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
 * @class wifi_actuator_
 * @ingroup cosm hal
 *
 * @brief WIFI actuator wrapper for the following supported robots:
 *
 * - ARGoS footbot. These robots will use wifi to broadcast data every timestep
 *   to all robots in range until told to do otherwise.
 * - NULL robot (robot without wifi capabilities). This is used to compile out
 *   the selected robot's actuator, and as such does not have a preprocessor
 *   definition.
 */
template<typename T>
class _wifi_actuator {
 public:
  explicit _wifi_actuator(T* const wifi) : m_wifi(wifi) {}

  /**
   * @brief Start broadcasting the specified data to all footbots within range.
   */
  template <typename U = T,
            RCPPSW_SFINAE_FUNC(detail::is_argos_rab_actuator<U>::value)>
  void broadcast_start(const struct wifi_packet& packet) {
    for (size_t i = 0; i < packet.data.size(); ++i) {
      m_wifi->SetData(i, packet.data[i]);
    } /* for(i..) */
  }

  /**
   * @brief Stop broadcasting the previously specified data to all footbots
   * within range.
   */
  template <typename U = T,
            RCPPSW_SFINAE_FUNC(detail::is_argos_rab_actuator<U>::value)>
  void broadcast_stop(void) {
    m_wifi->ClearData();
  }

  /**
   * @brief Reset the wifi device.
   */
  template <typename U = T,
            RCPPSW_SFINAE_FUNC(detail::is_argos_rab_actuator<U>::value)>
  void reset(void) {
    if (nullptr != m_wifi) {
      m_wifi->ClearData();
    }
  }

 private:
  /* clang-format off */
  T* const m_wifi;
  /* clang-format on */
};

#if HAL_CONFIG == HAL_CONFIG_ARGOS_FOOTBOT
using wifi_actuator = _wifi_actuator<argos::CCI_RangeAndBearingActuator>;
#endif /* HAL_CONFIG */

NS_END(actuators, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ACTUATORS_WIFI_ACTUATOR_HPP_ */
