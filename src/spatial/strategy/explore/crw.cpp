/**
 * \file crw.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/explore/crw.hpp"

#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
NS_START(cosm, spatial, strategy, explore);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
crw::crw(const csfsm::fsm_params* params,
         const cssexplore::config::explore_config* config,
         rmath::rng* rng)
    : base_explore(params, config, rng),
      ER_CLIENT_INIT("cosm.spatial.strategy.explore.crw") {}

/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
void crw::task_execute(void) {
  base_explore::task_execute();
  ER_DEBUG("Explore time=%zu,min_duration=%zu",
           steps().v(),
           config()->min_duration.v());

  /* apply wander force */
  wander();

  /* update nest zone tracking */
  nz_update();

  /* Set LEDs in accordance with obstacle detection */
  auto diag = saa()->actuation()->diagnostics();
  if (handle_ca()) {
    diag->emit(chactuators::diagnostics::ekEXP_INTERFERENCE);
  } else {
    diag->emit(chactuators::diagnostics::ekEXPLORE);
  }
} /* task_execute() */

void crw::task_reset(void) {
  base_explore::task_reset();

  /* no longer running */
  m_task_running = false;

  /*
   * If we happened to be experiencing interference, reset that state. This is
   * needed so if we are wandering and experiencing interference, AND trigger a
   * change to FSM state which results in not executing CRW for a while, when we
   * come back to CRW again we probably won't be experiencing interference, so
   * we don't want to report that we are.
   */
  inta_tracker()->state_reset();

  nz_tracker()->state_reset();
} /* task_reset() */

NS_END(explore, strategy, spatial, cosm);
