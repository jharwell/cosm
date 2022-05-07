/**
 * \file wander_random_thresh.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
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
