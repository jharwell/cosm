/**
 * \file base_flocking.cpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/flocking/base_flocking.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::flocking {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_flocking::base_flocking(const cssflocking::config::flocking_config* config,
                             const csfsm::fsm_params* params,
                             rmath::rng* rng)
    : base_strategy(params, rng),
      mc_config(*config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

} /* namespace cosm::spatial::strategy::flocking */
