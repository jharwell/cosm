/**
 * \file sensor_map.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
using sensor_map = std::unordered_map<std::type_index,
                                      sensor_variant<TSensorTypes...>
                                      >;

NS_END(subsystem, hal, cosm);
