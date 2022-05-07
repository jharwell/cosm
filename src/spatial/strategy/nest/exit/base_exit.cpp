/**
 * \file base_exit.cpp
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
#include "cosm/spatial/strategy/nest/exit/base_exit.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, exit);

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

NS_END(exit, nest, strategy, spatial, cosm);
