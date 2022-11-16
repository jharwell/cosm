/**
 * \file base_block_drop.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/blocks/drop/base_drop.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, blocks, drop);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_drop::base_drop(const csfsm::fsm_params* params,
                     const cssblocks::config::drop_config* config,
                     rmath::rng* rng)
    : base_strategy(params, rng), mc_config(*config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

NS_END(drop, blocks, strategy, spatial, cosm);
