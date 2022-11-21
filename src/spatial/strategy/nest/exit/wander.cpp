/**
 * \file wander.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/nest/exit/wander.hpp"

#include "cosm/subsystem/base_saa_subsystem.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::exit {

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

} /* namespace cosm::strategy::spatial::nest::exit */
