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

#ifndef INCLUDE_COSM_HAL_ARGOS_ACTUATORS_WIFI_ACTUATOR_HPP_
#define INCLUDE_COSM_HAL_ARGOS_ACTUATORS_WIFI_ACTUATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/utils/color.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/argos/actuators/argos_actuator.hpp"
#include "cosm/hal/wifi_packet.hpp"

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#endif /* COSM_HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_RangeAndBearingActuator;
} /* namespace argos */

NS_START(cosm, hal, argos, actuators, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename Actuator>
using is_rab_actuator = std::is_same<Actuator,
                                           ::argos::CCI_RangeAndBearingActuator>;
NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class wifi_actuator_impl
 * \ingroup hal argos actuators
 *
 * \brief WIFI actuator wrapper.
 *
 * Supports the following robots:
 *
 * - ARGoS footbot. These robots will use wifi to broadcast data every timestep
 *                  to all robots in range until told to do otherwise.
 *
 * - ARGoS epuck.  These robots will use wifi to broadcast data every timestep
 *                 to all robots in range until told to do otherwise.
 *
 * \tparam TActuator The underlying actuator handle type abstracted away by the
 *                   HAL. If nullptr, then that effectively disables the
 *                   actuator at compile time, and SFINAE ensures no member
 *                   functions can be called.
 */
template<typename TActuator>
class wifi_actuator_impl final : public rer::client<wifi_actuator_impl<TActuator>>,
                                 public chargos::actuators::argos_actuator<TActuator> {
 private:
  using chargos::actuators::argos_actuator<TActuator>::decoratee;

 public:
  using impl_type = TActuator;
  using chargos::actuators::argos_actuator<impl_type>::enable;
  using chargos::actuators::argos_actuator<impl_type>::disable;
  using chargos::actuators::argos_actuator<impl_type>::is_enabled;

  explicit wifi_actuator_impl(TActuator* const wifi)
      : ER_CLIENT_INIT("cosm.hal.argos.actuators.wifi"),
        chargos::actuators::argos_actuator<TActuator>(wifi) {}

  const wifi_actuator_impl& operator=(const wifi_actuator_impl&) = delete;
  wifi_actuator_impl(const wifi_actuator_impl&) = default;

  /**
   * \brief Reset the wifi device.
   */
  void reset(void) override {
    broadcast_stop();
    argos_actuator<TActuator>::reset();
  }

  /**
   * \brief Start broadcasting the specified data to all footbots within range.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_rab_actuator<U>::value)>
  bool broadcast_start(const struct wifi_packet& packet) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_CHECK(is_enabled(),
             "%s called when disabled",
             __FUNCTION__);

    for (size_t i = 0; i < packet.data.size(); ++i) {
      decoratee()->SetData(i, packet.data[i]);
    } /* for(i..) */

    return true;

 error:
    return false;
  }

  /**
   * \brief Stop broadcasting the previously specified data to all footbots
   * within range.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_rab_actuator<U>::value)>
  bool broadcast_stop(void) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_CHECK(is_enabled(),
             "%s called when disabled",
             __FUNCTION__);

    decoratee()->ClearData();
    return true;

 error:
    return false;
  }
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using wifi_actuator = wifi_actuator_impl<::argos::CCI_RangeAndBearingActuator>;
#else
class wifi_actuator {};
#endif /* COSM_HAL_TARGET */

NS_END(actuators, argos, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ARGOS_ACTUATORS_WIFI_ACTUATOR_HPP_ */
