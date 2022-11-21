/**
 * \file base_acq.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/nest/acq/base_acq.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::acq {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_acq::base_acq(const cssnest::config::acq_config* config,
                   const csfsm::fsm_params* params,
                   rmath::rng* rng)
    : base_strategy(params, rng),
      mc_config(*config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

} /* namespace cosm::spatial::strategy::nest::acq */
