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

#include "cosm/subsystem/base_saa_subsystem.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy {

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
  saa()->apf().accum(
      saa()->apf().phototaxis(light->readings()));
} /* phototaxis() */

void base_strategy::anti_phototaxis(void) {
  auto* light = saa()->sensing()->light();
  saa()->apf().accum(
      saa()->apf().anti_phototaxis(light->readings()));
} /* anti_phototaxis() */

void base_strategy::wander(void) {
  saa()->apf().accum(saa()->apf().wander(rng()));
} /* wander() */

bool base_strategy::handle_ca(void) {
  auto* prox = saa()->sensing()->proximity();

  if (auto obs = prox->avg_prox_obj()) {
    if (nullptr != inta_tracker()) {
      inta_tracker()->state_enter();
    }

    saa()->apf().accum(saa()->apf().avoidance(*obs));
    return true;
  } else {
    if (nullptr != inta_tracker()) {
      inta_tracker()->state_exit();
    }
  }
  return false;
} /* handle_ca() */

bool base_strategy::nz_update(void) {
  auto env = saa()->sensing()->env();

  if (env->detect(chsensors::env_sensor::kNestTarget)) {
    nz_tracker()->state_enter();
    return true;
  } else {
    nz_tracker()->state_exit();
  }
  return false;
} /* nz_update() */

} /* namespace cosm::spatial::strategy */
