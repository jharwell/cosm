/**
 * \file random_thresh.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/nest/acq/random_thresh.hpp"

#include "cosm/spatial/fsm/point_argument.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, acq);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
random_thresh::random_thresh(const cssnest::config::acq_config* config,
                             const csfsm::fsm_params* params,
                             rmath::rng* rng)
    : base_acq(config, params, rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void random_thresh::task_start(cta::taskable_argument* arg) {
  m_nest_loc = static_cast<fsm::point_argument*>(arg)->point();

  auto dist_to_light = (saa()->sensing()->rpos2D() - m_nest_loc).length();
  m_thresh = rspatial::euclidean_dist(rng()->uniform(0.01, dist_to_light));
  m_task_running = true;
} /* task_start() */

void random_thresh::task_execute(void) {
  /*
   * We might get pushed out of the nest by collision avoidance after initially
   * entering it.
   */
  auto env = saa()->sensing()->env();
  if (env->detect(chsensors::env_sensor::kNestTarget)) {
    auto dist_to_light = (saa()->sensing()->rpos2D() - m_nest_loc).length();
    if (dist_to_light <= m_thresh.v()) {
      m_task_running = false;
    }
  }

  phototaxis();
  handle_ca();
} /* task_execute() */

NS_END(acq, nest, spatial, strategy, cosm);
