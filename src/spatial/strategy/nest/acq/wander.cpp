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
#include "cosm/spatial/strategy/nest/acq/wander.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, acq);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
wander::wander(const cssnest::config::acq_config* config,
               const csfsm::fsm_params* params,
               rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.spatial.strategy.nest.acq.wander"),
      base_acq(config, params, rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void wander::task_start(cta::taskable_argument*) {
  task_reset();
  m_task_running = true;
} /* task_start() */

void wander::task_execute(void) {
  auto env = saa()->sensing()->env();

  ER_DEBUG("Nest detected: %d", env->detect(chsensors::env_sensor::kNestTarget));

  handle_ca();

  /*
   * We might get pushed out of the nest by collision avoidance after initially
   * entering it.
   */
  if (env->detect(chsensors::env_sensor::kNestTarget)) {
    ER_DEBUG("In nest: duration=%zu, threshold=%zu",
             m_steps.v(),
             config()->duration.v());

    base_strategy::wander();
    ++m_steps;
    /*
     * Once we are comfortably inside the nest, stop photoaxiing and just
     * wander.
     */
    if (m_steps <= config()->duration) {
      phototaxis();
    }
  } else { /* outside nest--just ca+phototaxis */
    phototaxis();
  }

  if (m_steps >= config()->duration) {
    m_task_running = false;
  }
} /* task_execute() */

NS_END(acq, nest, spatial, strategy, cosm);
