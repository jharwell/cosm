/**
 * \file util_hfsm.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
util_hfsm::util_hfsm(csubsystem::saa_subsystemQ3D* const saa,
                     rmath::rng* rng,
                     uint8_t max_states)
    : rpfsm::hfsm(max_states),
      ER_CLIENT_INIT("cosm.spatial.fsm.util_hfsm"),
      m_saa(saa),
      m_tracker(sensing()),
      m_rng(rng) {}

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_HFSM_ENTRY_DEFINE_ND(util_hfsm, entry_wait_for_signal) {
  actuation()->governed_diff_drive()->reset();
  actuation()->leds()->set_color(-1, rutils::color::kWHITE);
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

NS_END(fsm, spatial, cosm);
