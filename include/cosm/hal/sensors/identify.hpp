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
namespace cosm::hal::sensors {

/*******************************************************************************
 * Other Templates
 ******************************************************************************/
template <typename TSensor>
using is_null_sensor = std::is_null_pointer<TSensor>;

} /* namespace cosm::hal::sensors */
