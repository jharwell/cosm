/**
 * \file actuator_map.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <variant>
#include <typeindex>
#include <unordered_map>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::subsystem {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
template<typename ...TActuatorTypes>
using actuator_variant = std::variant<TActuatorTypes...>;

template<typename ...TActuatorTypes>
using actuator_variant_map = std::unordered_map<std::type_index,
                                                actuator_variant<TActuatorTypes...>
                                                >;

} /* namespace cosm::hal::subsystem */
