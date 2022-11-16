/**
 * \file backup.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/blocks/drop/backup.hpp"

#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, blocks, drop);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
backup::backup(const csfsm::fsm_params* params,
               const cssblocks::config::drop_config* config,
               rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.spatial.strategy.blocks.drop"),
      base_drop(params, config, rng),
      m_duration(config->duration) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void backup::task_start(cta::taskable_argument*) {
  task_reset();
  m_task_running = true;
  m_odom_start = saa()->sensing()->odometry()->reading();

  /* steering forces don't work for going backwards */
  saa()->apf2D().disable();

  /*
   * When we are backing up, we might be bringing a block to the nest, and will
   * erroneously "detect" it after dropping if sensing blocks is enabled
   */
  saa()->sensing()->env()->disable(chsensors::env_sensor::kBlockTarget);
} /* task_start() */

void backup::task_execute(void) {
  auto drive = saa()->actuation()->governed_diff_drive();

  ER_DEBUG("Backing up: steps=%zu,duration=%zu,speed=%f",
           m_steps.v(),
           m_duration.v(),
           drive->max_linear_speed());

  ckin::twist reverse = m_odom_start.twist;

  reverse.linear.x(-drive->max_linear_speed());
  reverse.angular.z(0);

  drive->fsm_drive(reverse);

  ++m_steps;

  if (m_steps >= m_duration) {
    task_reset();
  }
} /* task_execute() */

void backup::task_reset(void) {
  m_task_running = false;
  m_steps = rtypes::timestep(0);
  saa()->apf2D().enable();
  saa()->sensing()->env()->enable(chsensors::env_sensor::kBlockTarget);
}

NS_END(drop, blocks, spatial, strategy, cosm);
