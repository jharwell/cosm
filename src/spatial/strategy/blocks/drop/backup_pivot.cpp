/**
 * \file backup_pivot.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/blocks/drop/backup_pivot.hpp"

#include "cosm/subsystem/actuation_subsystem.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::blocks::drop {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
backup_pivot::backup_pivot(const csfsm::fsm_params* params,
               const cssblocks::config::drop_config* config,
               rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.spatial.strategy.blocks.drop"),
      base_drop(params, config, rng),
      m_duration(config->duration) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void backup_pivot::task_start(cta::taskable_argument*) {
  task_reset();
  m_task_running = true;
  m_odom_start = saa()->sensing()->odometry()->reading();

  /* steering forces don't work for going backwards */
  saa()->apf().disable();

  /*
   * When we are backing up, we might be bringing a block to the nest, and will
   * erroneously "detect" it after dropping if sensing blocks is enabled
   */
  saa()->sensing()->env()->disable(chsensors::env_sensor::kBlockTarget);
} /* task_start() */

void backup_pivot::task_execute(void) {
  auto drive = saa()->actuation()->governed_diff_drive();

  ER_DEBUG("Backing up: steps=%zu,duration=%zu,speed=%f",
           m_steps.v(),
           m_duration.v(),
           drive->max_linear_speed());

  if (m_steps < m_duration.v() / 2) {
    ckin::twist reverse = m_odom_start.twist;

    reverse.linear.x(-drive->max_linear_speed());
    reverse.angular.z(0);

    drive->fsm_drive(reverse);
  } else {
    ckin::twist pivot = m_odom_start.twist;
    pivot.linear.x(-drive->max_linear_speed() * 0.75);
    pivot.angular.z(-0.5);

    drive->fsm_drive(pivot);
  }

  ++m_steps;

  if (m_steps >= m_duration) {
    task_reset();
  }
} /* task_execute() */

void backup_pivot::task_reset(void) {
  m_task_running = false;
  m_steps = rtypes::timestep(0);
  saa()->apf().enable();
  saa()->sensing()->env()->enable(chsensors::env_sensor::kBlockTarget);
}

} /* namespace cosm::spatial::strategy::blocks::drop */
