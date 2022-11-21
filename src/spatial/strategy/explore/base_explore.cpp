/**
 * \file base_explore.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/explore/base_explore.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::explore {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_explore::base_explore(const csfsm::fsm_params* params,
                           const cssexplore::config::explore_config* config,
                           rmath::rng* rng)
    : base_strategy(params, rng), mc_config(*config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

} /* namespace cosm::spatial::strategy::explore */
