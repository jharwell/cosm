/**
 * \file backup.cpp
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
#include "cosm/spatial/strategy/block_drop/backup.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, block_drop);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
backup::backup(const csfsm::fsm_params* params,
               const config::block_drop_config* config,
               rmath::rng* rng)
    : base_block_drop(params, config, rng),
      m_duration(config->duration) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void backup::task_start(cta::taskable_argument*) {
  task_reset();
  m_task_running = true;
} /* task_start() */

void backup::task_execute(void) {
  handle_ca();

  ckin::twist reverse;
  auto drive = saa()->actuation()->governed_diff_drive();
  reverse.linear = rmath::vector3d::X * -drive->max_speed();

  drive->fsm_drive(reverse);
  ++m_steps;

  if (m_steps >= m_duration) {
    m_task_running = false;
  }
} /* task_execute() */

NS_END(block_drop, spatial, strategy, cosm);
