/**
 * \file util_hfsm.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/fsm/util_hfsm.hpp"

#include <numeric>

#include "cosm/spatial/fsm/util_signal.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
util_hfsm::util_hfsm(const fsm_params* const params,
                     rmath::rng* rng,
                     uint8_t max_states)
    : rpfsm::hfsm(max_states),
      ER_CLIENT_INIT("cosm.spatial.fsm.util_hfsm"),
      m_saa(params->saa),
      m_inta_tracker(params->inta),
      m_nz_tracker(params->nz),
      m_rng(rng) {}

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_HFSM_ENTRY_DEFINE_ND(util_hfsm, entry_wait_for_signal) {
  actuation()->governed_diff_drive()->reset();
  actuation()->diagnostics()->emit(chactuators::diagnostics::ekWAIT_FOR_SIGNAL);
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
rmath::radians util_hfsm::random_angle(void) {
  return rmath::radians(m_rng->uniform(0.0, rmath::radians::kPI.v()));
} /* randomize_vector_angle() */

csubsystem::sensing_subsystemQ3D* util_hfsm::sensing(void) {
  return m_saa->sensing();
}

const csubsystem::sensing_subsystemQ3D* util_hfsm::sensing(void) const {
  return m_saa->sensing();
}

csubsystem::actuation_subsystem2D* util_hfsm::actuation(void) {
  return m_saa->actuation();
}

csubsystem::actuation_subsystem2D* util_hfsm::actuation(void) const {
  return m_saa->actuation();
}

void util_hfsm::init(void) {
  hfsm::init();
  m_inta_tracker->state_reset();
  m_nz_tracker->state_reset();
} /* init() */

NS_END(fsm, spatial, cosm);
