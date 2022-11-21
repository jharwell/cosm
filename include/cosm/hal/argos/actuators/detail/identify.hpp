/**
 * \file identify.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_DifferentialSteeringActuator;
class CCI_PiPuckDifferentialDriveActuator;
class CCI_LEDsActuator;
class CCI_DirectionalLEDsActuator;
class CCI_RangeAndBearingActuator;
class CCI_DroneLEDsActuator;
} /* namespace argos */

namespace cosm::hal::argos::actuators::detail {

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TActuator>
using is_generic_ds_actuator = std::is_same<
  TActuator,
  ::argos::CCI_DifferentialSteeringActuator
  >;

template<typename TActuator>
using is_pipuck_ds_actuator = std::is_same<
  TActuator,
  ::argos::CCI_PiPuckDifferentialDriveActuator
  >;

template<typename TActuator>
using is_generic_led_actuator = std::is_same<
  TActuator,
  ::argos::CCI_LEDsActuator
  >;

template<typename TActuator>
using is_drone_led_actuator = std::is_same<
  TActuator,
  ::argos::CCI_DroneLEDsActuator
  >;

template<typename TActuator>
using is_generic_dirled_actuator = std::is_same<
  TActuator,
  ::argos::CCI_DirectionalLEDsActuator
  >;

template<typename Actuator>
using is_rab_actuator = std::is_same<
  Actuator,
  ::argos::CCI_RangeAndBearingActuator
  >;

} /* namespace cosm::hal::argos::actuators::detail */
