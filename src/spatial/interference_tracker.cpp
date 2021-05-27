/**
 * \file interference_tracker.cpp
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
#include "cosm/spatial/interference_tracker.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial);

/*******************************************************************************
 * Interference Metrics
 ******************************************************************************/
bool interference_tracker::exp_interference(void) const {
  return m_exp_interference;
} /* exp_interference() */

bool interference_tracker::entered_interference(void) const {
  return m_entered_interference;
} /* entered_interference() */

bool interference_tracker::exited_interference(void) const {
  return m_exited_interference;
} /* exited_interference() */

rtypes::timestep interference_tracker::interference_duration(void) const {
  if (m_exited_interference) {
    return mc_sensing->tick() - m_interference_start;
  }
  return rtypes::timestep(0);
} /* interference_duration() */

rmath::vector3z interference_tracker::interference_loc3D(void) const {
  return mc_sensing->dpos3D();
} /* interference_loc2D() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void interference_tracker::inta_enter(void) {
  if (!m_exp_interference) {
    if (!m_entered_interference) {
      m_entered_interference = true;
      m_interference_start = mc_sensing->tick();
    }
  } else {
    m_entered_interference = false;
  }
  m_exp_interference = true;
} /* inta_enter() */

void interference_tracker::inta_exit(void) {
  if (!m_exited_interference) {
    if (m_exp_interference) {
      m_exited_interference = true;
    }
  } else {
    m_exited_interference = false;
  }
  m_exp_interference = false;
  m_entered_interference = false; /* catches 1 timestep interferences correctly */
} /* inta_exit() */

void interference_tracker::inta_reset(void) {
  m_entered_interference = false;
  m_exited_interference = false;
  m_exp_interference = false;
  m_interference_start = rtypes::timestep(0);
} /* inta_reset() */

NS_END(spatial, cosm);
