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
#include "cosm/flocking/stoch_fov.hpp"

#include "cosm/subsystem/base_saa_subsystem.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
namespace cosm::flocking {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stoch_fov::stoch_fov(const cflocking::config::flocking_config* config,
                     const csfsm::fsm_params* params,
                     rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.flocking.stoch_fov"),
      base_flocking(config, params, rng),
      m_prob(config->stoch_fov.theta_max,
             config->stoch_fov.mean_interaction_dist) {}


/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
void stoch_fov::task_start(cta::taskable_argument*) {
  /*
   * From the paper, we need to set initial conditions to give each agent some
   * velocity to bootstrap the algorithm. They say to sample U[0, v_c], but this
   * works just fine.
   */
  wander();
  ++m_init_count;

  m_task_running = true;
} /* task_start() */

void stoch_fov::task_execute(void) {

  auto vel = saa()->sensing()->odometry()->reading().twist.linear.to_2D();
  auto pos = saa()->sensing()->odometry()->reading().pose.position.to_2D();

  /*
   * Initial conditions need to be extended a bit more so things will
   * stabilize.
   */
  if (++m_init_count <= 10) {
    wander();
    return;
  }
  /*
   * Not an error--can happen in large arenas with small neighborhood radius.
   */
  if (m_odom.empty()) {
    wander();
    return;
  }

  handle_ca();

  auto& apf = saa()->apf();
  auto& self = apf.entity();

  /* We assume we only get neighbors/no self references, so no checking */

  std::vector<double> probs(m_odom.size());
  for (size_t i = 0; i < m_odom.size(); ++i) {
    /* calculate interaction probability */
    auto prob = m_prob(&self, i, m_odom);
    ER_DEBUG("Calculated interaction probability=%.10e for agent%zu", prob, i);
    probs[i] = prob;
  } /* for(i..) */

  /*
   * Start at a random point in the probability vector so things are as unbiased
   * as possible.
   */
  auto start = rng()->uniform(rmath::rangez(0, m_odom.size() - 1));
  for (size_t i = 0; i < m_odom.size(); ++i) {
    auto idx = (i + start)  % m_odom.size();

    if (rng()->bernoulli(probs[idx])) {
      auto other_odom = m_odom[idx];
      auto other_zorient = rmath::radians(other_odom.pose.orientation.z());
      auto other_vel =  rmath::vector2d(other_odom.twist.linear.length(),
                                        (other_zorient));

      /*
       * If the other agent isn't really moving, ignore them. Trying to flock
       * with agents doing hard turns doesn't work.
       */
      if (other_vel.length() < 0.0001) {
        continue;
      }

      ER_DEBUG("Interacting with agent%zu,vel=%s@%s",
               idx,
               rcppsw::to_string(other_vel).c_str(),
               rcppsw::to_string(other_vel.angle()).c_str());

      /* align with chosen neighbor */
      apf.accum(apf.alignment({other_vel}));

      /* maintain constant speed */
      apf.accum(apf.constant_speed({other_vel}));
      return;
    }
  } /* for(i..) */

  /*
   * No agent found to interact with--travel in a random (mostly) random
   * direction
   */
  ER_DEBUG("No agent found to interact with--random walk");
  wander();
} /* task_execute() */

void stoch_fov::task_reset(void) {
  base_flocking::task_reset();

  /* no longer running */
  m_task_running = false;

  m_init_count = 0;
} /* task_reset() */

} /* namespace cosm::flocking */
