/**
 * \file base_exit.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/nest/exit/base_exit.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::exit {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_exit::base_exit(const cssnest::config::exit_config* config,
                     const csfsm::fsm_params* params,
                     rmath::rng* rng)
    : base_strategy(params, rng),
      mc_config(*config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

} /* namespace cosm::spatial::strategy::nest::exit */
