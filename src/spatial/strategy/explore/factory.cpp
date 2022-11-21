/**
 * \file factory.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/explore/factory.hpp"

#include "cosm/spatial/strategy/explore/crw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::explore {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
factory::factory(void) { register_type<crw>(kCRW); }

} /* namespace cosm::spatial::strategy::explore */
