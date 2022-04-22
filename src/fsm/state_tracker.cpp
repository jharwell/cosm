/**
 * \file state_tracker.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "cosm/fsm/state_tracker.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, fsm);

/*******************************************************************************
 * State Metrics
 ******************************************************************************/
bool state_tracker::in_state(void) const { return m_in_state; } /* in_state() */

bool state_tracker::entered_state(void) const {
  return m_entered_state;
} /* entered_state() */

bool state_tracker::exited_state(void) const {
  return m_exited_state;
} /* exited_state() */

rtypes::timestep state_tracker::state_duration(void) const {
  if (m_exited_state) {
    return mc_sensing->tick() - m_state_start;
  }
  return rtypes::timestep(0);
} /* state_duration() */

rmath::vector3z state_tracker::state_loc3D(void) const {
  return mc_sensing->dpos3D();
} /* state_loc2D() */

rtypes::timestep state_tracker::state_entry_time(void) const {
  return m_state_start;
} /* state_entry_time() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void state_tracker::state_enter(void) {
  if (!m_in_state) {
    if (!m_entered_state) {
      m_entered_state = true;
      m_state_start = mc_sensing->tick();
    }
  } else {
    m_entered_state = false;
  }
  m_in_state = true;
} /* state_enter() */

void state_tracker::state_exit(void) {
  if (!m_exited_state) {
    if (m_in_state) {
      m_exited_state = true;
    }
  } else {
    m_exited_state = false;
  }
  m_in_state = false;
  m_entered_state = false; /* catches 1 timestep states correctly */
} /* state_exit() */

void state_tracker::state_reset(void) {
  m_entered_state = false;
  m_exited_state = false;
  m_in_state = false;
  m_state_start = rtypes::timestep(0);
} /* state_reset() */

NS_END(fsm, cosm);
