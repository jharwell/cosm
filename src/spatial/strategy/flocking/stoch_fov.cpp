/**
 * \file stoch_fov.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/flocking/stoch_fov.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
namespace cosm::spatial::strategy::flocking {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stoch_fov::stoch_fov(const cssflocking::config::flocking_config* config,
                     const csfsm::fsm_params* params,
                     rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.spatial.strategy.flocking.stoch_fov"),
      base_flocking(config, params, rng),
      m_prob(config->stoch_fov.theta_max,
             config->stoch_fov.mean_interaction_dist) {}


/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
void stoch_fov::task_execute(void) {
  ER_ASSERT(!m_boids.empty(), "FATAL: No agents provided");

  auto vel = saa()->sensing()->odometry()->reading().twist.linear.to_2D();
  auto pos = saa()->sensing()->odometry()->reading().pose.position.to_2D();

  /* pick agent to interact with */
  auto idx = rng()->uniform(rmath::rangez(0, m_boids.size() - 1));

  /* calculate interaction probability */
  auto prob = m_prob(pos, vel, idx, m_boids);

  if (rng()->bernoulli(prob)) {
    /* align with chosen neighbor */
    saa()->apf2D().alignment({m_boids[idx]->odometry().twist.linear.to_2D()});

    /* maintain constant speed */
    saa()->apf2D().constant_speed({m_boids[idx]->odometry().twist.linear.to_2D()});
  } else {
    /* travel in a random (mostly) random direction */
    wander();
  }
} /* task_execute() */

void stoch_fov::task_reset(void) {
  base_flocking::task_reset();

  /* no longer running */
  m_task_running = false;
} /* task_reset() */

} /* namespace cosm::spatial::strategy::flocking */
