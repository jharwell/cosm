/**
 * \file base_strategy.cpp
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
#include "cosm/spatial/strategy/base_strategy.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_strategy::base_strategy(const csfsm::fsm_params* params, rmath::rng* rng)
    : m_saa(params->saa),
      m_inta_tracker(params->inta),
      m_nz_tracker(params->nz),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_strategy::phototaxis(void) {
  auto* light = saa()->sensing()->light();
  saa()->steer_force2D().accum(
      saa()->steer_force2D().phototaxis(light->readings()));
} /* phototaxis() */

void base_strategy::wander(void) {
  saa()->steer_force2D().accum(saa()->steer_force2D().wander(rng()));
} /* wander() */

bool base_strategy::handle_ca(void) {
  auto* prox = saa()->sensing()->proximity();

  if (auto obs = prox->avg_prox_obj()) {
    inta_tracker()->state_enter();
    saa()->steer_force2D().accum(saa()->steer_force2D().avoidance(*obs));
    return true;
  } else {
    inta_tracker()->state_exit();
    return false;
  }
} /* handle_ca() */

bool base_strategy::nz_update(void) {
  if (saa()->sensing()->nest_detect()) {
    nz_tracker()->state_enter();
    return true;
  } else {
    nz_tracker()->state_exit();
    return false;
  }
} /* nz_update() */

NS_END(strategy, spatial, cosm);
