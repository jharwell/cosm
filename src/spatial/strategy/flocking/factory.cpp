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
#include "cosm/spatial/strategy/flocking/factory.hpp"

#include "cosm/spatial/strategy/flocking/stoch_fov.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::flocking {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
factory::factory(void) { register_type<stoch_fov>(kStochFOV); }

} /* namespace cosm::spatial::strategy::flocking */
