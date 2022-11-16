/**
 * \file base_strategy.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/base_strategy.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

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

void base_strategy::anti_phototaxis(void) {
  auto* light = saa()->sensing()->light();
  saa()->steer_force2D().accum(
      saa()->steer_force2D().anti_phototaxis(light->readings()));
} /* anti_phototaxis() */

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
  auto env = saa()->sensing()->env();

  if (env->detect(chsensors::env_sensor::kNestTarget)) {
    nz_tracker()->state_enter();
    return true;
  } else {
    nz_tracker()->state_exit();
    return false;
  }
} /* nz_update() */

NS_END(strategy, spatial, cosm);
