/**
 * \file wander_random_thresh.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/nest/acq/wander_random_thresh.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, acq);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
wander_random_thresh::wander_random_thresh(const cssnest::config::acq_config* config,
                                           const csfsm::fsm_params* params,
                                           rmath::rng* rng)
    : random_thresh(config, params, rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void wander_random_thresh::task_execute(void) {
  random_thresh::task_execute();
  wander();
} /* task_execute() */

NS_END(acq, nest, spatial, strategy, cosm);
