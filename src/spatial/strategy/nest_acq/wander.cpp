/**
 * \file wander.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "cosm/spatial/strategy/nest_acq/wander.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest_acq);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void wander::task_start(cta::taskable_argument*) {
  m_task_running = true;
  m_steps = rng()->uniform(kMIN_STEPS, kMAX_STEPS);
} /* task_start() */

void wander::task_execute(void) {
  auto* ground = saa()->sensing()->ground();

  handle_ca();

  /*
   * We might get pushed out of the nest by collision avoidance after initially
   * entering it.
   */
  if (ground->detect(hal::sensors::ground_sensor::kNestTarget)) {
    base_strategy::wander();
    m_count++;
    /*
     * Once we are comfortably inside the nest, stop phootoaxiing and just
     * wander.
     */
    if (m_count <= kMIN_STEPS) {
      phototaxis();
    }
  } else { /* outside nest--just ca+phototaxis */
    phototaxis();
  }

  if (m_count >= m_steps) {
    m_task_running = false;
  }
} /* task_execute() */

NS_END(nest_acq, spatial, strategy, cosm);
