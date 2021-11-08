/**
 * \file crw.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/spatial/strategy/explore/crw.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
NS_START(cosm, spatial, strategy, explore);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
crw::crw(const csfsm::fsm_params* params, rmath::rng* rng)
    : base_strategy(params, rng),
      ER_CLIENT_INIT("cosm.spatial.strategy.explore.crw") {}

/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
void crw::task_execute(void) {
  /* apply wander force */
  wander();

  /* update nest zone tracking */
  nz_update();

  /* Set LEDs in accordance with obstacle detection */
  if (handle_ca()) {
    saa()->actuation()->leds()->set_color(-1, rutils::color::kRED);
  } else {
    saa()->actuation()->leds()->set_color(-1, rutils::color::kMAGENTA);
  }
} /* task_execute() */

void crw::task_reset(void) {
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
