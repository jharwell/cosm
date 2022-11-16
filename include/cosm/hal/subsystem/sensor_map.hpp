/**
 * \file sensor_map.hpp
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
NS_START(cosm, hal, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
template<typename ...TSensorTypes>
using sensor_variant = std::variant<TSensorTypes...>;

template<typename ...TSensorTypes>
using sensor_variant_map = std::unordered_map<std::type_index,
                                              sensor_variant<TSensorTypes...>
                                              >;

NS_END(subsystem, hal, cosm);
