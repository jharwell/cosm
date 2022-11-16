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
#include "cosm/spatial/strategy/nest/exit/factory.hpp"

#include "cosm/spatial/strategy/nest/exit/anti_phototaxis.hpp"
#include "cosm/spatial/strategy/nest/exit/wander.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, exit);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
factory::factory(void) {
  register_type<wander>(kWander);
  register_type<anti_phototaxis>(kAntiPhototaxis);
}

NS_END(exit, nest, strategy, spatial, cosm);
