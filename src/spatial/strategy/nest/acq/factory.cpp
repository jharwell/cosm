/**
 * \file factory.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/nest/acq/factory.hpp"

#include "cosm/spatial/strategy/nest/acq/random_thresh.hpp"
#include "cosm/spatial/strategy/nest/acq/wander.hpp"
#include "cosm/spatial/strategy/nest/acq/wander_random_thresh.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::acq {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
factory::factory(void) {
  register_type<wander>(kWander);
  register_type<random_thresh>(kRandomThresh);
  register_type<wander_random_thresh>(kWanderRandomThresh);
}

} /* namespace cosm::spatial::strategy::nest::acq */
