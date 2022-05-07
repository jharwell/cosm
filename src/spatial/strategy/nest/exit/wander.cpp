/**
 * \file wander.cpp
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
#include "cosm/spatial/strategy/nest/exit/wander.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, exit);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
wander::wander(const cssnest::config::exit_config* config,
               const csfsm::fsm_params* params,
               rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.spatial.strategy.nest.exit.wander"),
      base_exit(config, params, rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void wander::task_start(cta::taskable_argument*) {
  task_reset();
  m_task_running = true;
} /* task_start() */

void wander::task_execute(void) {
  auto env = saa()->sensing()->env();

  handle_ca();

  base_strategy::wander();

  if (!env->detect(chsensors::env_sensor::kNestTarget)) {
    m_task_running = false;
  }
} /* task_execute() */

NS_END(exit, nest, spatial, strategy, cosm);
